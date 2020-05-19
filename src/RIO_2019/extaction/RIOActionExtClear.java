package RIO_2019.extaction;

import adf.agent.action.Action;
import adf.agent.action.common.ActionMove;
import adf.agent.action.common.ActionRest;
import adf.agent.action.police.ActionClear;
import adf.agent.communication.MessageManager;
import adf.agent.develop.DevelopData;
import adf.agent.info.AgentInfo;
import adf.agent.info.ScenarioInfo;
import adf.agent.info.WorldInfo;
import adf.agent.module.ModuleManager;
import adf.agent.precompute.PrecomputeData;
import adf.component.extaction.ExtAction;
import adf.component.module.algorithm.Clustering;
import adf.component.module.algorithm.PathPlanning;
import rescuecore2.config.NoSuchConfigOptionException;
import rescuecore2.misc.Pair;
import rescuecore2.misc.geometry.GeometryTools2D;
import rescuecore2.misc.geometry.Line2D;
import rescuecore2.misc.geometry.Point2D;
import rescuecore2.misc.geometry.Vector2D;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;

import java.awt.*;
import java.util.*;
import java.util.List;
import java.util.stream.Collectors;

import static rescuecore2.standard.entities.StandardEntityURN.REFUGE;

public class RIOActionExtClear extends ExtAction {
    private PathPlanning pathPlanning;

    private int clearDistance;
    private int forcedMove;
    private int thresholdRest;
    private int kernelTime;

    private EntityID target;
    private Map<EntityID, Set<Point2D>> movePointCache;
    private int oldClearX;
    private int oldClearY;
    private int count;

    //for calcClear
    private ArrayList<Pair<Integer, Integer>> removedQueue;
    private final int queueLength = 10;
    private int checkedTime = -1;

    //for calcSearch
    private Clustering clustering;
    private ArrayList<Integer> movedTime;
    private boolean stopped;
    private ArrayList<EntityID> unsearchedBuildingIDs;
    private int clusterIndex;
    private int changeClusterCycle;
    protected Random random;
    private ArrayList<Point2D> previousLocations;
    private ArrayList<List<EntityID>> previousPaths;
    private ArrayList<EntityID> previousTarget;

    public RIOActionExtClear(AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager,
                             DevelopData developData) {
        super(ai, wi, si, moduleManager, developData);
        this.clearDistance = si.getClearRepairDistance();
        this.forcedMove = developData.getInteger("ActionExtClear.forcedMove", 3);
        this.thresholdRest = developData.getInteger("ActionExtClear.rest", 100);

        this.target = null;
        this.movePointCache = new HashMap<>();
        this.oldClearX = 0;
        this.oldClearY = 0;
        this.count = 0;

        switch (si.getMode()) {
            case PRECOMPUTATION_PHASE:
                this.pathPlanning = moduleManager.getModule("ActionExtClear.PathPlanning",
                        "adf.sample.module.algorithm.SamplePathPlanning");
                this.clustering = moduleManager.getModule("ActionExtClear.Clustering.Police",
                        "RIO_2019.module.algorithm.RioneKmeansPP");
                break;
            case PRECOMPUTED:
                this.pathPlanning = moduleManager.getModule("ActionExtClear.PathPlanning",
                        "adf.sample.module.algorithm.SamplePathPlanning");
                this.clustering = moduleManager.getModule("ActionExtClear.Clustering.Police",
                        "RIO_2019.module.algorithm.RioneKmeansPP");
                break;
            case NON_PRECOMPUTE:
                this.pathPlanning = moduleManager.getModule("ActionExtClear.PathPlanning",
                        "adf.sample.module.algorithm.SamplePathPlanning");
                this.clustering = moduleManager.getModule("ActionExtClear.Clustering.Police",
                        "RIO_2019.module.algorithm.RioneKmeansPP");
                break;
        }

        // calcSearch用
        this.clustering = moduleManager.getModule("ActionTransport.Clustering.Ambulance",
                "RIO_2019.module.algorithm.RioneKmeansPP");
        unsearchedBuildingIDs = new ArrayList<>();
        movedTime = new ArrayList<>();
        this.changeClusterCycle = 5;
        this.clusterIndex = 0;
        this.random = new Random();
        this.stopped = false;
        this.previousLocations = new ArrayList<>();
        this.previousPaths = new ArrayList<>();
        this.previousTarget = new ArrayList<>();

        removedQueue = new ArrayList<>();
    }

    @Override
    public ExtAction precompute(PrecomputeData precomputeData) {
        super.precompute(precomputeData);
        if (this.getCountPrecompute() >= 2) {
            return this;
        }
        this.pathPlanning.precompute(precomputeData);
        try {
            this.kernelTime = this.scenarioInfo.getKernelTimesteps();
        } catch (NoSuchConfigOptionException e) {
            this.kernelTime = -1;
        }
        return this;
    }

    @Override
    public ExtAction resume(PrecomputeData precomputeData) {
        super.resume(precomputeData);
        if (this.getCountResume() >= 2) {
            return this;
        }
        this.pathPlanning.resume(precomputeData);
        try {
            this.kernelTime = this.scenarioInfo.getKernelTimesteps();
        } catch (NoSuchConfigOptionException e) {
            this.kernelTime = -1;
        }
        return this;
    }

    @Override
    public ExtAction preparate() {
        super.preparate();
        if (this.getCountPreparate() >= 2) {
            return this;
        }
        this.pathPlanning.preparate();
        try {
            this.kernelTime = this.scenarioInfo.getKernelTimesteps();
        } catch (NoSuchConfigOptionException e) {
            this.kernelTime = -1;
        }
        return this;
    }

    @Override
    public ExtAction updateInfo(MessageManager messageManager) {
        super.updateInfo(messageManager);
        if (this.getCountUpdateInfo() >= 2) {
            return this;
        }
        this.pathPlanning.updateInfo(messageManager);

        if (this.unsearchedBuildingIDs.isEmpty()) {
            this.reset();
        }

        // 未探索建物の精査
        List<EntityID> perceivedBuildings = new ArrayList<>();// 見つけた建物
        for (EntityID id : worldInfo.getChanged().getChangedEntities()) {
            StandardEntity se = worldInfo.getEntity(id);
            if (se instanceof Building) {
                perceivedBuildings.add(id);
            }
        }
        for (EntityID pID : perceivedBuildings) {
            unsearchedBuildingIDs.remove(pID);
        }
        return this;
    }

    @Override
    public ExtAction setTarget(EntityID target) {
        this.target = null;
        StandardEntity entity = this.worldInfo.getEntity(target);
        if (entity != null) {
            if (entity instanceof Road) {
                this.target = target;
            } else if (entity.getStandardURN().equals(StandardEntityURN.BLOCKADE)) {
                this.target = ((Blockade) entity).getPosition();
            } else if (entity instanceof Building) {
                this.target = target;
            }
            previousTarget.add(target);
        }
        return this;
    }

    /**
     * @author 天田(Amada)
     * @since 2018
     * clearActionするときは必ずcalcClearActionメソッドを通すこと
     */
    @Override
    public ExtAction calc() {
        this.result = null;

        PoliceForce policeForce = (PoliceForce) this.agentInfo.me();

        // SakaeなどPFのClearでバグが出る場合の処理
        StandardEntity me = this.agentInfo.me();
        if (isInExtendedBlockade((Human) me)) {
            StandardEntity pos = this.worldInfo.getPosition(me.getID());
            if (pos instanceof Road) {
                Blockade blockade = blockadeWithHuman((Human) me);
                if (blockade != null) {
                    this.result = new ActionClear(blockade);
                    return this;
                }
            }
        }
        if (this.needRest(policeForce)) {
            List<EntityID> list = new ArrayList<>();
            if (this.target != null) {
                list.add(this.target);
            }
            this.result = this.calcRest(policeForce, this.pathPlanning, list);
            if (this.result != null) {
                return this;
            }
        }

        this.result = this.calcRefuge();
        if (this.result != null) {
            return this;
        }

        this.result = this.calcGroup();
        if (this.result != null) {
            return this;
        }


        EntityID agentPosition = policeForce.getPosition();
        StandardEntity targetEntity = this.worldInfo.getEntity(this.target);
        StandardEntity positionEntity = Objects.requireNonNull(this.worldInfo.getEntity(agentPosition));
        if (!(targetEntity instanceof Area)) {
            return this;
        }
        if (positionEntity instanceof Road) {

            // 拡張した瓦礫に挟まってるエージェントがいるならそのエージェントを助ける
            helpAgentInExtendedBlockade();
            // 拡張した瓦礫に挟まってるエージェントがいるならそのエージェントの座標へ移動
            moveToAgentInExtendedBlockade();
            // 瓦礫に挟まってるエージェントがいるならそのエージェントを助ける
            helpAgentInBlockade();

            if (this.result != null) {
                return this;
            }
        }

        if (agentPosition.equals(this.target)) {
            this.result = this.getAreaClearAction(policeForce, targetEntity);
        } else if (((Area) targetEntity).getEdgeTo(agentPosition) != null) {
            this.result = this.getNeighbourPositionAction(policeForce, (Area) targetEntity);
        } else {
            List<EntityID> path = this.pathPlanning.getResult(agentPosition, this.target);
            if (path != null && path.size() > 0) {
                int index = path.indexOf(agentPosition);
                if (index == -1) {
                    Area area = (Area) positionEntity;
                    for (int i = 0; i < path.size(); i++) {
                        if (area.getEdgeTo(path.get(i)) != null) {
                            index = i;
                            break;
                        }
                    }
                } else {
                    index++;
                }
                if (index >= 0 && index < (path.size())) {
                    StandardEntity entity = this.worldInfo.getEntity(path.get(index));
                    if (entity != null) {
                        this.result = this.getNeighbourPositionAction(policeForce, (Area) entity);
                    }
                }
            }
        }

        if (this.result != null) {
            return this;
        }

        this.result = calcSearch();
        return this;
    }

    //避難所付近の瓦礫を取り除く
    private Action calcRefuge() {
        PoliceForce policeForce = (PoliceForce) this.agentInfo.me();
        ArrayList<StandardEntity> refuges = new ArrayList<>(this.worldInfo.getEntitiesOfType(StandardEntityURN.REFUGE));//全ての「避難所」
        ArrayList<StandardEntity> roads = new ArrayList<>(this.worldInfo.getEntitiesOfType(StandardEntityURN.ROAD));//全ての「道」
        ArrayList<StandardEntity> roadsInBlockade = new ArrayList<>();//全ての「避難所付近の瓦礫で塞がれた道」

        for (StandardEntity entityRe : refuges) {
            for (StandardEntity entityRo : roads) {
                if (entityRo instanceof Road) {
                    Road road = (Road) entityRo;
                    //「道」が「避難所」付近にあり、瓦礫で塞がれている場合
                    if (worldInfo.getDistance(entityRe, entityRo) <= 10000 && road.isBlockadesDefined() && road.getBlockades().size() > 0) {
                        //全ての「避難所付近の瓦礫で塞がれた道」を特定
                        roadsInBlockade.add(entityRo);
                    }
                }
            }
        }

        StandardEntity Target = null;

        //「避難所付近の瓦礫で塞がれた道」が存在する場合
        if (!roadsInBlockade.isEmpty()) {
            for (StandardEntity entityRIB : roadsInBlockade) {
                //エージェントが「避難所付近の瓦礫で塞がれた道」付近にいる場合
                if (worldInfo.getDistance(policeForce, entityRIB) <= clearDistance / 2) {
                    return calcClearAction(policeForce, ((Road) entityRIB).getX(), ((Road) entityRIB).getY());
                }
                //いない場合
                else {
                    Target = entityRIB;
                }
            }
        }

        if (Target != null) {
            //「避難所付近の瓦礫で塞がれた道」への道を特定
            return getMoveAction(pathPlanning,
                    policeForce.getPosition(),
                    Target.getID(),
                    ((Road) Target).getX(),
                    ((Road) Target).getY());
        }

        return null;
    }

    //大勢のエージェントが通る道の瓦礫を取り除く
    private Action calcGroup() {
        PoliceForce policeForce = (PoliceForce) this.agentInfo.me();
        ArrayList<StandardEntity> roads = new ArrayList<>(this.worldInfo.getEntitiesOfType(StandardEntityURN.ROAD));//全ての「道」
        ArrayList<StandardEntity> fireBrigades = new ArrayList<>(this.worldInfo.getEntitiesOfType(StandardEntityURN.FIRE_BRIGADE));//全ての「FB」
        ArrayList<StandardEntity> ambulanceTeams = new ArrayList<>(this.worldInfo.getEntitiesOfType(StandardEntityURN.AMBULANCE_TEAM));//全ての「AT」
//        ArrayList<StandardEntity> policeForces = new ArrayList<>(this.worldInfo.getEntitiesOfType(StandardEntityURN.POLICE_FORCE));//全ての「PF」
        ArrayList<StandardEntity> civilians = new ArrayList<>(this.worldInfo.getEntitiesOfType(StandardEntityURN.CIVILIAN));//全ての「市民」
        ArrayList<Pair<StandardEntity, List<Human>>> roadsHaveAgents = new ArrayList<>();//全ての「多くのエージェントがいる道」

        for (StandardEntity entityR : roads) {
            if (entityR instanceof Road) {
                Road road = (Road) entityR;
                ArrayList<Human> humans = new ArrayList<>();
                //「道」が瓦礫で塞がれている場合
                if (road.isBlockadesDefined() && road.getBlockades().size() > 0) {
                    for (StandardEntity entityF : fireBrigades) {
                        //「道」付近に「FB」がいる場合
                        if (isInExtendedRoad((Human) entityF, road)) {
                            //「道」付近にいる全ての「FB」を数える
                            humans.add((Human) entityF);
                        }
                    }
                    for (StandardEntity entityA : ambulanceTeams) {
                        //「道」付近に「AT」がいる場合
                        if (isInExtendedRoad((Human) entityA, road)) {
                            //「道」付近にいる全ての「AT」を数える
                            humans.add((Human) entityA);
                        }
                    }
                    for (StandardEntity entityC : civilians) {
                        //「道」付近に「市民」がいる場合
                        if (isInExtendedRoad((Human) entityC, road)) {
                            //「道」付近にいる全ての「市民」を数える
                            humans.add((Human) entityC);
                        }
                    }
                    //「道」に多くの「エージェント」がいる場合（とりあえず5人以上）
                    if (humans.size() >= 5) {
                        //全ての「多くのエージェントがいる道」を特定
                        roadsHaveAgents.add(new Pair<>(entityR, humans));
                    }
                }
            }
        }

        StandardEntity Target = null;
        int dx = 0, dy = 0;
        roadsHaveAgents.sort(new HumanOnRoadComparator());

        //「多くのエージェントがいる道」が存在する場合
        if (!roadsHaveAgents.isEmpty()) {
            for (Pair<StandardEntity, List<Human>> entityRHC : roadsHaveAgents) {
                //エージェントが「多くのエージェントがいる道」付近にいる場合
                if (worldInfo.getDistance(policeForce,entityRHC.second().get(0)) < clearDistance ) {
                    return calcClearAction(policeForce, entityRHC.second().get(0).getX(), entityRHC.second().get(0).getY());
                }
                //いない場合
                else {
                    Target = entityRHC.first();
                    dx = entityRHC.second().get(0).getX();
                    dy = entityRHC.second().get(0).getY();
                }
            }
        }

        if (Target != null) {
            return getMoveAction(pathPlanning,
                    policeForce.getPosition(),
                    Target.getID(),
                    dx,
                    dy);
        }

        return null;
    }


    /**
     * 指定したHumanが瓦礫に挟まっているならtrueを返す関数
     *
     * @param human human
     * @return true or false
     * @author 岡島(Okajima)
     * @author 天田(Amada)
     */
    private boolean isInBlockade(Human human) {
        if (!human.isXDefined() || !human.isYDefined())
            return false;
        int agentX = human.getX();
        int agentY = human.getY();
        StandardEntity positionEntity = this.worldInfo.getPosition(human);
        if (positionEntity instanceof Road) {
            Road road = (Road) positionEntity;
            if (road.isBlockadesDefined() && road.getBlockades().size() > 0) {
                for (Blockade blockade : worldInfo.getBlockades(road)) {
                    if (blockade.getShape().contains(agentX, agentY)) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    /**
     * 瓦礫に挟まってるエージェントがいるならそのエージェントを助ける関数
     *
     * @author 岡島(Okajima)
     * @author 天田(Amada)
     */
    private void helpAgentInBlockade() {
        Set<EntityID> changedEntities = this.worldInfo.getChanged().getChangedEntities();
        StandardEntity caughtAgent = changedEntities.stream()
                .map(id -> this.worldInfo.getEntity(id))
                .filter(se -> se instanceof AmbulanceTeam || se instanceof FireBrigade)
                .filter(se -> isInBlockade((Human) se))
                .findFirst().orElse(null);

        if (caughtAgent != null) {
            // 規定以内のエージェントに挟まったエージェントがいるなら助ける
            int distance = this.worldInfo.getDistance(this.agentInfo.me().getID(), caughtAgent.getID());
            if (0 < distance && distance < this.clearDistance) {
                this.result = calcClearAction((PoliceForce) agentInfo.me(), ((Human) caughtAgent).getX(), ((Human) caughtAgent).getY());
            } else {
                this.result = getMoveAction(pathPlanning,
                        Objects.requireNonNull(this.worldInfo.getPosition(this.agentInfo.me().getID())).getID(),
                        Objects.requireNonNull(this.worldInfo.getPosition(caughtAgent.getID())).getID(),
                        ((Human) caughtAgent).getX(), ((Human) caughtAgent).getY());
            }
        }
    }

    /**
     * 拡張した瓦礫に挟まってるエージェントがいるならそのエージェントを助ける関数
     *
     * @author 岡島
     */
    private void helpAgentInExtendedBlockade() {
        Set<EntityID> changedEntities = this.worldInfo.getChanged().getChangedEntities();
        StandardEntity caughtAgent = changedEntities.stream()
                .map(id -> this.worldInfo.getEntity(id))
                .filter(se -> se instanceof AmbulanceTeam || se instanceof FireBrigade)
                .filter(se -> isInExtendedBlockade((Human) se))
                .findFirst().orElse(null);

        if (caughtAgent != null) {
            // 規定以内のエージェントに挟まったエージェントがいるなら助ける 距離もう少し長くても良いか
            int distance = this.worldInfo.getDistance(this.agentInfo.me().getID(), caughtAgent.getID());
            if (0 < distance && distance < this.clearDistance) {
                this.result = calcClearAction((PoliceForce) agentInfo.me(), ((Human) caughtAgent).getX(), ((Human) caughtAgent).getY());
            }
        }
    }

    /**
     * 拡張した瓦礫に挟まってるエージェントがいるならそのエージェントの座標に重なるように動く
     *
     * @author 岡島(Okajima)
     * @author 天田(Amada)
     */
    private void moveToAgentInExtendedBlockade() {
        Set<EntityID> changedEntities = this.worldInfo.getChanged().getChangedEntities();
        // 瓦礫内部にはいないが瓦礫に挟まったエージェント
        StandardEntity caughtAgentNotIn = changedEntities.stream()
                .map(id -> this.worldInfo.getEntity(id))
                .filter(se -> se instanceof AmbulanceTeam || se instanceof FireBrigade)
                .filter(se -> isInExtendedBlockade((Human) se))
                .filter(se -> !isInBlockade((Human) se))
                .findFirst().orElse(null);

        if (caughtAgentNotIn != null) {
            // 規定以内のエージェントに挟まったエージェントがいるなら移動
            int distance = this.worldInfo.getDistance(this.agentInfo.me().getID(), caughtAgentNotIn.getID());
            //System.out.println(distance);

            if (1000 < distance && distance < this.clearDistance) { // 1以内だとすでに重なっていると見なす
                int myPosX = ((Human) this.agentInfo.me()).getX();
                int myPosY = ((Human) this.agentInfo.me()).getY();
                int agentPosX = ((Human) caughtAgentNotIn).getX();
                int agentPosY = ((Human) caughtAgentNotIn).getY();

                // 移動先との間に瓦礫がある
                if (existsBlockade(myPosX, myPosY, agentPosX, agentPosY)) {
                    return;
                }
                this.result = getMoveAction(pathPlanning,
                        Objects.requireNonNull(this.worldInfo.getPosition(this.agentInfo.me().getID())).getID(),
                        Objects.requireNonNull(this.worldInfo.getPosition((Human) caughtAgentNotIn)).getID(),
                        agentPosX,
                        agentPosY);
            }
        }
    }

    /**
     * 二点間に瓦礫があるならtrueを返す関数(10等分点で考えるので距離が伸びると精度が落ちる)
     *
     * @param fromX fromY toX toY
     * @return true or false
     * @author 岡島
     */
    private boolean existsBlockade(int fromX, int fromY, int toX, int toY) {
        Set<EntityID> changedEntities = this.worldInfo.getChanged().getChangedEntities();
        Set<Blockade> blockades = new HashSet<>();
        changedEntities.stream()
                .filter(id -> this.worldInfo.getEntity(id) instanceof Blockade)
                .forEach(id -> blockades.add((Blockade) this.worldInfo.getEntity(id)));

        for (int i = 1; i < 10; i++) {
            int px = (fromX * i + toX * (10 - i)) / 10;
            int py = (fromY * i + toY * (10 - i)) / 10;
            for (Blockade blockade : blockades) {
                if (blockade.getShape().contains(px, py)) {
                    return true;
                }
            }
        }

        return false;
    }

    /**
     * fromからみてto方向の手前に瓦礫があるかどうか
     *
     * @param fromX fromY toX toY
     * @return true or false
     * @author 岡島
     */
    private boolean existsBlockadeNear(int fromX, int fromY, int toX, int toY) {
        Set<EntityID> changedEntities = this.worldInfo.getChanged().getChangedEntities();
        Set<Blockade> blockades = new HashSet<>();
        changedEntities.stream()
                .filter(id -> this.worldInfo.getEntity(id) instanceof Blockade)
                .forEach(id -> blockades.add((Blockade) this.worldInfo.getEntity(id)));

        int px = (fromX * 9 + toX * 1) / 10;
        int py = (fromY * 9 + toY * 1) / 10;
        for (Blockade blockade : blockades) {
            if (blockade.getShape().contains(px, py)) {
                return true;
            }
        }

        return false;
    }

    /**
     * 指定したHumanが瓦礫に挟まっているならそのBlockadeを返す関数
     *
     * @param human 指定するhuman
     * @return blockade or null
     * @author 岡島
     */
    private Blockade blockadeWithHuman(Human human) {
        if (!human.isXDefined() || !human.isYDefined())
            return null;
        int agentX = human.getX();
        int agentY = human.getY();
        StandardEntity positionEntity = this.worldInfo.getPosition(human);
        if (positionEntity instanceof Road) {
            Road road = (Road) positionEntity;
            if (road.isBlockadesDefined() && road.getBlockades().size() > 0) {
                for (Blockade blockade : worldInfo.getBlockades(road)) {
                    if (blockade.getShape().contains(agentX, agentY)) {
                        return blockade;
                    }
                }
            }
        }
        return null;
    }


    /**
     * 指定ポイント方向に向かってシナリオに定義されたClearDistanceにベクトルを再調整してActionClearを返す
     * 座標x,yからも出せるように
     *
     * @param targetX target X
     * @param targetY target Y
     * @return ActionClear
     * @author 岡島(Okajima)
     * @author 天田(Amada)
     */
    private Action calcClearAction(PoliceForce policeForce, int targetX, int targetY) {
        Point2D agentPoint = toPoint2D(policeForce.getX(), policeForce.getY());
        Point2D targetPoint = new Point2D((double) targetX, (double) targetY);
        Vector2D targetVector = toVector2D(agentPoint, targetPoint).normalised().scale(this.scenarioInfo.getClearRepairDistance());
        targetPoint = toPoint2D(agentPoint, targetVector);
        int fixedTargetX = (int) targetPoint.getX();
        int fixedTargetY = (int) targetPoint.getY();return new ActionClear(fixedTargetX, fixedTargetY);
    }


    /**
     * 指定したHumanが拡張した瓦礫に挟まっているならtrueを返す関数
     *
     * @return true or false
     * @author 岡島(Okajima)
     * @author 天田(Amada)
     */
    private boolean isInExtendedBlockade(Human human) {
        if (!human.isXDefined() || !human.isYDefined()) return false;
        int agentX = human.getX();
        int agentY = human.getY();
        StandardEntity positionEntity = this.worldInfo.getPosition(human);
        if (positionEntity instanceof Road) {
            Road road = (Road) positionEntity;
            if (road.isBlockadesDefined() && road.getBlockades().size() > 0) {
                for (Blockade blockade : worldInfo.getBlockades(road)) {
                    Shape extendedShape = getExtendedShape(blockade);
                    if (extendedShape != null && extendedShape.contains(agentX, agentY)) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    /**
     * 指定したHumanが拡張したRoadの上ならtrueを返す関数
     *
     * @param human human
     * @return true or false
     * @author 岡島(Okajima)
     * @author 天田(Amada)
     */
    private boolean isInExtendedRoad(Human human, Road road) {
        if (!human.isXDefined() || !human.isYDefined()) return false;
        int agentX = human.getX();
        int agentY = human.getY();

        Shape extendedShape = getExtendedShape(road, 100);
        return (extendedShape != null && extendedShape.contains(agentX, agentY));
    }

    private Action calcSearch() {
        if (agentInfo.getTime() < scenarioInfo.getKernelAgentsIgnoreuntil()) {
            return null;
        }

        int size = previousTarget.size();
        if(size > 2 && previousTarget.get(size-1).equals(previousTarget.get(size-2))){
            return null;
        }

        return getSearchAction(pathPlanning, this.agentInfo.getPosition(), this.unsearchedBuildingIDs);
    }

    private Action getSearchAction(PathPlanning pathPlanning, EntityID from, Collection<EntityID> targets) {
        pathPlanning.setFrom(from);
        pathPlanning.setDestination(targets);
        List<EntityID> path = pathPlanning.calc().getResult();
        previousPaths.add(path);

        if (previousPaths.size() < 2 || !isStopped(previousPaths.get(0), previousPaths.get(1))) {
            if (path != null && path.size() > 0) {
                StandardEntity entity = this.worldInfo.getEntity(path.get(path.size() - 1));
                if (entity instanceof Building) {
                    if (entity.getStandardURN() != StandardEntityURN.REFUGE) {
                        path.remove(path.size() - 1);
                    }
                }
                movedTime.add(agentInfo.getTime());// 動いた時のTimeを記録

                if (path.size() > 1)
                    return new ActionMove(path);
            }
            return null;
        }
        this.stopped = true;
        reset();
        return null;
    }

    // 止まってる判定はtrue、止まってなければfalse
    private boolean isStopped(List<EntityID> path1, List<EntityID> path2) {
        Human agent = (Human) this.agentInfo.me();
        previousLocations.add(new Point2D(agent.getX(), agent.getY()));// 移動するときの場所を記録(0が現在地)

        if (path1 == null || path2 == null) {
            return false;
        }
        if (path1.size() != path2.size()) {
            return false;
        } else {
            for (int i = 0; i < path1.size(); i++) {
                EntityID id1 = path1.get(i);
                EntityID id2 = path2.get(i);
                if (!id1.equals(id2))
                    return false;
            }
        }

        if (previousLocations.size() > 2) {
            return withinRange(previousLocations.get(0), previousLocations.get(1), previousLocations.get(2));
        }
        return false;
    }

    private boolean withinRange(Point2D position1, Point2D position2, Point2D position3) {
        int range = 50000;

        double dist1 = GeometryTools2D.getDistance(position1, position2);
        double dist2 = GeometryTools2D.getDistance(position1, position3);

        return dist1 < range && dist2 < range;

    }

    private void reset() {
        this.unsearchedBuildingIDs.clear();
        this.previousPaths.clear();
        this.previousLocations.clear();

        if ((this.agentInfo.getTime() != 0 && (this.agentInfo.getTime() % this.changeClusterCycle) == 0) || stopped) {
            this.stopped = false;
            this.clusterIndex = random.nextInt(clustering.getClusterNumber());
            this.changeClusterCycle = random.nextInt(16) + 15;// 変更

        }
        Collection<StandardEntity> clusterEntities = new ArrayList<>();
        if (clustering != null) {
            clusterEntities.addAll(this.clustering.getClusterEntities(clusterIndex));
        }

        if (clusterEntities.size() > 0) {
            for (StandardEntity entity : clusterEntities) {
                if (entity instanceof Building && entity.getStandardURN() != REFUGE) {
                    this.unsearchedBuildingIDs.add(entity.getID());
                }
            }
        } else {
            this.unsearchedBuildingIDs.addAll(this.worldInfo.getEntityIDsOfType(StandardEntityURN.BUILDING));
        }
    }

    // nishida
    private Action getMoveAction(PathPlanning pathPlanning, EntityID from, EntityID target, int x, int y) {
        pathPlanning.setFrom(from);
        pathPlanning.setDestination(target);
        List<EntityID> path = pathPlanning.calc().getResult();
        previousPaths.add(path);

        if (previousPaths.size() < 2 || !isStopped(previousPaths.get(0), previousPaths.get(1))) {// 止まってるかどうか判定
            if (path != null && path.size() > 0) {
                StandardEntity entity = this.worldInfo.getEntity(path.get(path.size() - 1));
                if (entity instanceof Building) {
                    if (entity.getStandardURN() != StandardEntityURN.REFUGE) {
                        path.remove(path.size() - 1);
                    }
                }

                try {
                    int fromX = Objects.requireNonNull(worldInfo.getLocation(from)).first();
                    int fromY = Objects.requireNonNull(worldInfo.getLocation(from)).second();
                    int toX = Objects.requireNonNull(worldInfo.getLocation(path.get(path.size() - 1))).first();
                    int toY = Objects.requireNonNull(worldInfo.getLocation(path.get(path.size() - 1))).second();

                    if (existsBlockadeNear(fromX, fromY, toX, toY)){
                        return calcClearAction((PoliceForce) (agentInfo.me()), toX, toY);
                    }
                }catch (NullPointerException e){

                }

                movedTime.add(agentInfo.getTime());// 動いた時のTimeを記録
                return new ActionMove(path, x, y);
            }
        }

        return null;
    }

    private Action getMoveAction(PathPlanning pathPlanning, EntityID from, Collection<EntityID> target, int x, int y) {
        pathPlanning.setFrom(from);
        pathPlanning.setDestination(target);
        List<EntityID> path = pathPlanning.calc().getResult();
        previousPaths.add(path);

        if (previousPaths.size() < 2 || !isStopped(previousPaths.get(0), previousPaths.get(1))) {// 止まってるかどうか判定
            if (path != null && path.size() > 0) {
                StandardEntity entity = this.worldInfo.getEntity(path.get(path.size() - 1));
                if (entity instanceof Building) {
                    if (entity.getStandardURN() != StandardEntityURN.REFUGE) {
                        path.remove(path.size() - 1);
                    }
                }
                movedTime.add(agentInfo.getTime());// 動いた時のTimeを記録
                return new ActionMove(path, x, y);
            }

        }

        return null;
    }

    private Action getMoveAction(PathPlanning pathPlanning, EntityID from, EntityID target) {
        pathPlanning.setFrom(from);
        pathPlanning.setDestination(target);
        List<EntityID> path = pathPlanning.calc().getResult();
        previousPaths.add(path);

        if (previousPaths.size() < 2 || !isStopped(previousPaths.get(0), previousPaths.get(1))) {// 止まってるかどうか判定
            if (path != null && path.size() > 0) {
                StandardEntity entity = this.worldInfo.getEntity(path.get(path.size() - 1));
                if (entity instanceof Building) {
                    if (entity.getStandardURN() != StandardEntityURN.REFUGE) {
                        path.remove(path.size() - 1);
                    }
                }
                movedTime.add(agentInfo.getTime());// 動いた時のTimeを記録
                return new ActionMove(path);
            }
        }
        return null;
    }

    private Action getMoveAction(PathPlanning pathPlanning, EntityID from, Collection<EntityID> target) {
        pathPlanning.setFrom(from);
        pathPlanning.setDestination(target);
        List<EntityID> path = pathPlanning.calc().getResult();
        previousPaths.add(path);

        if (previousPaths.size() < 2 || !isStopped(previousPaths.get(0), previousPaths.get(1))) {// 止まってるかどうか判定
            if (path != null && path.size() > 0) {
                StandardEntity entity = this.worldInfo.getEntity(path.get(path.size() - 1));
                if (entity instanceof Building) {
                    if (entity.getStandardURN() != StandardEntityURN.REFUGE) {
                        path.remove(path.size() - 1);
                    }
                }
                movedTime.add(agentInfo.getTime());// 動いた時のTimeを記録
                return new ActionMove(path);
            }
        }
        return null;
    }


    private Action getAreaClearAction(PoliceForce police, StandardEntity targetEntity) {
        if (targetEntity instanceof Building) {
            return null;
        }
        Road road = (Road) targetEntity;
        if (!road.isBlockadesDefined() || road.getBlockades().isEmpty()) {
            return null;
        }
        Collection<Blockade> blockades = this.worldInfo.getBlockades(road).stream().filter(Blockade::isApexesDefined)
                .collect(Collectors.toSet());
        int minDistance = Integer.MAX_VALUE;
        Blockade clearBlockade = null;
        for (Blockade blockade : blockades) {
            for (Blockade another : blockades) {
                if (!blockade.getID().equals(another.getID()) && this.intersect(blockade, another)) {
                    int distance1 = this.worldInfo.getDistance(police, blockade);
                    int distance2 = this.worldInfo.getDistance(police, another);
                    if (distance1 <= distance2 && distance1 < minDistance) {
                        minDistance = distance1;
                        clearBlockade = blockade;
                    } else if (distance2 < minDistance) {
                        minDistance = distance2;
                        clearBlockade = another;
                    }
                }
            }
        }
        if (clearBlockade != null) {
            if (minDistance < this.clearDistance*0.8) {
                return new ActionClear(clearBlockade);
            } else {
                return getMoveAction(pathPlanning, police.getPosition(), road.getID(), clearBlockade.getX(),
                        clearBlockade.getY());
            }
        }
        double agentX = police.getX();
        double agentY = police.getY();
        clearBlockade = null;
        double minPointDistance = Double.MAX_VALUE;
        int clearX = 0;
        int clearY = 0;
        for (Blockade blockade : blockades) {
            int[] apexes = blockade.getApexes();
            for (int i = 0; i < (apexes.length - 2); i += 2) {
                double distance = this.getDistance(agentX, agentY, apexes[i], apexes[i + 1]);
                if (distance < minPointDistance) {
                    clearBlockade = blockade;
                    minPointDistance = distance;
                    clearX = apexes[i];
                    clearY = apexes[i + 1];
                }
            }
        }
        if (clearBlockade != null) {
            if (minPointDistance < this.clearDistance*0.8) {
                Vector2D vector = this.scaleClear(this.getVector(agentX, agentY, clearX, clearY));
                clearX = (int) (agentX + vector.getX());
                clearY = (int) (agentY + vector.getY());
                return new ActionClear(clearX, clearY, clearBlockade);
            }
            return getMoveAction(pathPlanning, police.getPosition(), road.getID(), clearX, clearY);
        }
        return null;
    }

    private Action getNeighbourPositionAction(PoliceForce police, Area target) {
        double agentX = police.getX();
        double agentY = police.getY();
        StandardEntity position = Objects.requireNonNull(this.worldInfo.getPosition(police));
        Edge edge = target.getEdgeTo(position.getID());
        if (edge == null) {
            return null;
        }
        if (position instanceof Road) {
            Road road = (Road) position;
            if (road.isBlockadesDefined() && road.getBlockades().size() > 0) {
                double midX = (edge.getStartX() + edge.getEndX()) / 2;
                double midY = (edge.getStartY() + edge.getEndY()) / 2;
                if (this.intersect(agentX, agentY, midX, midY, road)) {
                    return this.getIntersectEdgeAction(agentX, agentY, edge, road);
                }
                ActionClear actionClear = null;
                boolean isMove = false;
                Vector2D vector = this.scaleClear(this.getVector(agentX, agentY, midX, midY));
                int clearX = (int) (agentX + vector.getX());
                int clearY = (int) (agentY + vector.getY());
                vector = this.scaleBackClear(vector);
                int startX = (int) (agentX + vector.getX());
                int startY = (int) (agentY + vector.getY());
                for (Blockade blockade : this.worldInfo.getBlockades(road)) {
                    if (blockade == null || !blockade.isApexesDefined()) {
                        continue;
                    }
                    if (this.intersect(startX, startY, midX, midY, blockade)) {
                        if (this.intersect(startX, startY, clearX, clearY, blockade)) {
                            if (actionClear == null) {
                                actionClear = new ActionClear(clearX, clearY, blockade);
                                if (this.equalsPoint(this.oldClearX, this.oldClearY, clearX, clearY)) {
                                    if (this.count >= this.forcedMove) {
                                        this.count = 0;
                                        return getMoveAction(pathPlanning, police.getPosition(), road.getID(), clearX,
                                                clearY);
                                    }
                                    this.count++;
                                }
                                this.oldClearX = clearX;
                                this.oldClearY = clearY;
                            } else {
                                if (actionClear.getTarget() != null) {
                                    Blockade another = (Blockade) this.worldInfo.getEntity(actionClear.getTarget());
                                    if (another != null && this.intersect(blockade, another)) {
                                        return new ActionClear(another);
                                    }
                                }
                                return actionClear;
                            }
                        } else if (!isMove) {
                            isMove = true;
                        }
                    }
                }
                if (actionClear != null) {
                    return actionClear;
                } else if (isMove) {
                    return getMoveAction(pathPlanning, police.getPosition(), road.getID(), (int) midX, (int) midY);
                }
            }
        }
        if (target instanceof Road) {
            Road road = (Road) target;
            if (!road.isBlockadesDefined() || road.getBlockades().isEmpty()) {
                return getMoveAction(pathPlanning, police.getPosition(), target.getID());
            }
            Blockade clearBlockade = null;
            double minPointDistance = Double.MAX_VALUE;
            int clearX = 0;
            int clearY = 0;
            for (EntityID id : road.getBlockades()) {
                Blockade blockade = (Blockade) this.worldInfo.getEntity(id);
                if (blockade != null && blockade.isApexesDefined()) {
                    int[] apexes = blockade.getApexes();
                    for (int i = 0; i < (apexes.length - 2); i += 2) {
                        double distance = this.getDistance(agentX, agentY, apexes[i], apexes[i + 1]);
                        if (distance < minPointDistance) {
                            clearBlockade = blockade;
                            minPointDistance = distance;
                            clearX = apexes[i];
                            clearY = apexes[i + 1];
                        }
                    }
                }
            }
            if (clearBlockade != null && minPointDistance < this.clearDistance*0.8) {
                Vector2D vector = this.scaleClear(this.getVector(agentX, agentY, clearX, clearY));
                clearX = (int) (agentX + vector.getX());
                clearY = (int) (agentY + vector.getY());
                if (this.equalsPoint(this.oldClearX, this.oldClearY, clearX, clearY)) {
                    if (this.count >= this.forcedMove) {
                        this.count = 0;
                        return getMoveAction(pathPlanning, police.getPosition(), road.getID(), clearX, clearY);
                    }
                    this.count++;
                }
                this.oldClearX = clearX;
                this.oldClearY = clearY;
                return new ActionClear(clearX, clearY, clearBlockade);
            }
        }
        return getMoveAction(pathPlanning, police.getPosition(), target.getID());
    }

    private Action getIntersectEdgeAction(double agentX, double agentY, Edge edge, Road road) {
        double midX = (edge.getStartX() + edge.getEndX()) / 2;
        double midY = (edge.getStartY() + edge.getEndY()) / 2;
        return this.getIntersectEdgeAction(agentX, agentY, midX, midY, road);
    }

    private Action getIntersectEdgeAction(double agentX, double agentY, double pointX, double pointY, Road road) {
        Set<Point2D> movePoints = this.getMovePoints(road);
        Point2D bestPoint = null;
        double bastDistance = Double.MAX_VALUE;
        for (Point2D p : movePoints) {
            if (!this.intersect(agentX, agentY, p.getX(), p.getY(), road)) {
                if (!this.intersect(pointX, pointY, p.getX(), p.getY(), road)) {
                    double distance = this.getDistance(pointX, pointY, p.getX(), p.getY());
                    if (distance < bastDistance) {
                        bestPoint = p;
                        bastDistance = distance;
                    }
                }
            }
        }
        if (bestPoint != null) {
            double pX = bestPoint.getX();
            double pY = bestPoint.getY();
            if (!road.isBlockadesDefined()) {
                return getMoveAction(pathPlanning, this.agentInfo.getPosition(), road.getID(), (int) pX, (int) pY);
            }
            ActionClear actionClear = null;
            Vector2D vector = this.scaleClear(this.getVector(agentX, agentY, pX, pY));
            int clearX = (int) (agentX + vector.getX());
            int clearY = (int) (agentY + vector.getY());
            vector = this.scaleBackClear(vector);
            int startX = (int) (agentX + vector.getX());
            int startY = (int) (agentY + vector.getY());
            boolean isMove = false;
            for (Blockade blockade : this.worldInfo.getBlockades(road)) {
                if (this.intersect(startX, startY, pX, pY, blockade)) {
                    if (this.intersect(startX, startY, clearX, clearY, blockade)) {
                        if (actionClear == null) {
                            actionClear = new ActionClear(clearX, clearY, blockade);
                        } else {
                            if (actionClear.getTarget() != null) {
                                Blockade another = (Blockade) this.worldInfo.getEntity(actionClear.getTarget());
                                if (another != null && this.intersect(blockade, another)) {
                                    return new ActionClear(another);
                                }
                            }
                            return actionClear;
                        }
                    } else if (!isMove) {
                        isMove = true;
                        // actionMove = new
                        // ActionMove(Lists.newArrayList(road.getID()), (int)
                        // pX, (int) pY);
                    }
                }
            }
            if (actionClear != null) {
                return actionClear;
            } else if (isMove) {
                return getMoveAction(pathPlanning, this.agentInfo.getPosition(), road.getID(), (int) pX, (int) pY);
                // return actionMove;
            }
        }
        Action action = this.getAreaClearAction((PoliceForce) this.agentInfo.me(), road);
        if (action == null) {
            action = getMoveAction(pathPlanning, this.agentInfo.getPosition(), road.getID(), (int) pointX,
                    (int) pointY);
            // action = new ActionMove(Lists.newArrayList(road.getID()), (int)
            // pointX, (int) pointY);
        }
        return action;
    }

    private boolean equalsPoint(double p1X, double p1Y, double p2X, double p2Y) {
        return this.equalsPoint(p1X, p1Y, p2X, p2Y, 1000.0D);
    }

    private boolean equalsPoint(double p1X, double p1Y, double p2X, double p2Y, double range) {
        return (p2X - range < p1X && p1X < p2X + range) && (p2Y - range < p1Y && p1Y < p2Y + range);
    }

    private boolean isInside(double pX, double pY, int[] apex) {
        Point2D p = new Point2D(pX, pY);
        Vector2D v1 = (new Point2D(apex[apex.length - 2], apex[apex.length - 1])).minus(p);
        Vector2D v2 = (new Point2D(apex[0], apex[1])).minus(p);
        double theta = this.getAngle(v1, v2);

        for (int i = 0; i < apex.length - 2; i += 2) {
            v1 = (new Point2D(apex[i], apex[i + 1])).minus(p);
            v2 = (new Point2D(apex[i + 2], apex[i + 3])).minus(p);
            theta += this.getAngle(v1, v2);
        }
        return Math.round(Math.abs((theta / 2) / Math.PI)) >= 1;
    }

    private boolean intersect(double agentX, double agentY, double pointX, double pointY, Area area) {
        for (Edge edge : area.getEdges()) {
            double startX = edge.getStartX();
            double startY = edge.getStartY();
            double endX = edge.getEndX();
            double endY = edge.getEndY();
            if (java.awt.geom.Line2D.linesIntersect(agentX, agentY, pointX, pointY, startX, startY, endX, endY)) {
                double midX = (edge.getStartX() + edge.getEndX()) / 2;
                double midY = (edge.getStartY() + edge.getEndY()) / 2;
                if (!equalsPoint(pointX, pointY, midX, midY) && !equalsPoint(agentX, agentY, midX, midY)) {
                    return true;
                }
            }
        }
        return false;
    }

    private boolean intersect(Blockade blockade, Blockade another) {
        if (blockade.isApexesDefined() && another.isApexesDefined()) {
            int[] apexes0 = blockade.getApexes();
            int[] apexes1 = another.getApexes();
            for (int i = 0; i < (apexes0.length - 2); i += 2) {
                for (int j = 0; j < (apexes1.length - 2); j += 2) {
                    if (java.awt.geom.Line2D.linesIntersect(apexes0[i], apexes0[i + 1], apexes0[i + 2], apexes0[i + 3],
                            apexes1[j], apexes1[j + 1], apexes1[j + 2], apexes1[j + 3])) {
                        return true;
                    }
                }
            }
            for (int i = 0; i < (apexes0.length - 2); i += 2) {
                if (java.awt.geom.Line2D.linesIntersect(apexes0[i], apexes0[i + 1], apexes0[i + 2], apexes0[i + 3],
                        apexes1[apexes1.length - 2], apexes1[apexes1.length - 1], apexes1[0], apexes1[1])) {
                    return true;
                }
            }
            for (int j = 0; j < (apexes1.length - 2); j += 2) {
                if (java.awt.geom.Line2D.linesIntersect(apexes0[apexes0.length - 2], apexes0[apexes0.length - 1],
                        apexes0[0], apexes0[1], apexes1[j], apexes1[j + 1], apexes1[j + 2], apexes1[j + 3])) {
                    return true;
                }
            }
        }
        return false;
    }

    private boolean intersect(double agentX, double agentY, double pointX, double pointY, Blockade blockade) {
        List<Line2D> lines = GeometryTools2D.pointsToLines(GeometryTools2D.vertexArrayToPoints(blockade.getApexes()),
                true);
        for (Line2D line : lines) {
            Point2D start = line.getOrigin();
            Point2D end = line.getEndPoint();
            double startX = start.getX();
            double startY = start.getY();
            double endX = end.getX();
            double endY = end.getY();
            if (java.awt.geom.Line2D.linesIntersect(agentX, agentY, pointX, pointY, startX, startY, endX, endY)) {
                return true;
            }
        }
        return false;
    }

    private double getDistance(double fromX, double fromY, double toX, double toY) {
        double dx = toX - fromX;
        double dy = toY - fromY;
        return Math.hypot(dx, dy);
    }

    private double getAngle(Vector2D v1, Vector2D v2) {
        double flag = (v1.getX() * v2.getY()) - (v1.getY() * v2.getX());
        double angle = Math
                .acos(((v1.getX() * v2.getX()) + (v1.getY() * v2.getY())) / (v1.getLength() * v2.getLength()));
        if (flag > 0) {
            return angle;
        }
        if (flag < 0) {
            return -1 * angle;
        }
        return 0.0D;
    }

    private Vector2D getVector(double fromX, double fromY, double toX, double toY) {
        return (new Point2D(toX, toY)).minus(new Point2D(fromX, fromY));
    }

    private Vector2D scaleClear(Vector2D vector) {
        return vector.normalised().scale(this.clearDistance);
    }

    private Vector2D scaleBackClear(Vector2D vector) {
        return vector.normalised().scale(-510);
    }

    private Set<Point2D> getMovePoints(Road road) {
        Set<Point2D> points = this.movePointCache.get(road.getID());
        if (points == null) {
            points = new HashSet<>();
            int[] apex = road.getApexList();
            for (int i = 0; i < apex.length; i += 2) {
                for (int j = i + 2; j < apex.length; j += 2) {
                    double midX = (apex[i] + apex[j]) / 2;
                    double midY = (apex[i + 1] + apex[j + 1]) / 2;
                    if (this.isInside(midX, midY, apex)) {
                        points.add(new Point2D(midX, midY));
                    }
                }
            }
            for (Edge edge : road.getEdges()) {
                double midX = (edge.getStartX() + edge.getEndX()) / 2;
                double midY = (edge.getStartY() + edge.getEndY()) / 2;
                points.remove(new Point2D(midX, midY));
            }
            this.movePointCache.put(road.getID(), points);
        }
        return points;
    }

    private boolean needRest(Human agent) {
        int hp = agent.getHP();
        int damage = agent.getDamage();
        if (damage == 0 || hp == 0) {
            return false;
        }
        int activeTime = (hp / damage) + ((hp % damage) != 0 ? 1 : 0);
        if (this.kernelTime == -1) {
            try {
                this.kernelTime = this.scenarioInfo.getKernelTimesteps();
            } catch (NoSuchConfigOptionException e) {
                //no change
                //this.kernelTime = -1;
            }
        }
        return damage >= this.thresholdRest || (activeTime + this.agentInfo.getTime()) < this.kernelTime;
    }

    private Action calcRest(Human human, PathPlanning pathPlanning, Collection<EntityID> targets) {
        EntityID position = human.getPosition();
        Collection<EntityID> refuges = this.worldInfo.getEntityIDsOfType(StandardEntityURN.REFUGE);
        int currentSize = refuges.size();
        if (refuges.contains(position)) {
            return new ActionRest();
        }
        List<EntityID> firstResult = null;
        while (refuges.size() > 0) {
            pathPlanning.setFrom(position);
            pathPlanning.setDestination(refuges);
            List<EntityID> path = pathPlanning.calc().getResult();
            if (path != null && path.size() > 0) {
                if (firstResult == null) {
                    firstResult = new ArrayList<>(path);
                    if (targets == null || targets.isEmpty()) {
                        break;
                    }
                }
                EntityID refugeID = path.get(path.size() - 1);
                pathPlanning.setFrom(refugeID);
                pathPlanning.setDestination(targets);
                List<EntityID> fromRefugeToTarget = pathPlanning.calc().getResult();
                if (fromRefugeToTarget != null && fromRefugeToTarget.size() > 0) {
                    return new ActionMove(path);
                }
                refuges.remove(refugeID);
                // remove failed
                if (currentSize == refuges.size()) {
                    break;
                }
                currentSize = refuges.size();
            } else {
                break;
            }
        }
        return firstResult != null ? new ActionMove(firstResult) : null;
    }

    /**
     * 指定座標をPoint2Dに変換する
     *
     * @return Area
     * @author 兼近
     */
    private Point2D toPoint2D(double x, double y) {
        return new Point2D(x, y);
    }

    /**
     * 始点から終点へのベクトルを作る
     *
     * @param startPoint start
     * @param endPoint end
     * @return Vector2D
     * @author 兼近
     */
    private Vector2D toVector2D(Point2D startPoint, Point2D endPoint) {
        return new Vector2D(endPoint.getX() - startPoint.getX(), endPoint.getY() - startPoint.getY());
    }

    /**
     * 指定座標に指定ベクトルを加算する
     *
     * @param point point
     * @param vector adding vector
     * @return 加算済み頂点
     */
    private Point2D toPoint2D(Point2D point, Vector2D vector) {
        return new Point2D((int) (point.getX() + vector.getX()), (int) (point.getY() + vector.getY()));
    }

    /**
     * 重心方向から各頂点方向に2だけ広げたShapeを返す.瓦礫に引っかかりにくくするための関数．
     *
     * @param blockade blockade
     * @return 重心方向から各頂点方向に2だけ広げたShapeを返す
     * @author 岡島
     */

    private Shape getExtendedShape(Blockade blockade) {
        Shape shape = null;
        int[] allApexes = blockade.getApexes();
        int count = allApexes.length / 2;
        int[] xs = new int[count];
        int[] ys = new int[count];
        double centerX = 0;
        double centerY = 0;
        for (int i = 0; i < count; ++i) {
            xs[i] = allApexes[i * 2];
            ys[i] = allApexes[i * 2 + 1];
            centerX += xs[i];
            centerY += ys[i];
        }
        centerX /= count;
        centerY /= count;
        for (int i = 0; i < count; ++i) {
            // 重心から頂点へのベクトル
            double vectorX = xs[i] - centerX;
            double vectorY = ys[i] - centerY;
            double magnitude = Math.sqrt(vectorX * vectorX + vectorY * vectorY); // ベクトルの大きさ
            // 重心から頂点への大きさ2のベクトルを頂点に足して四捨五入
            xs[i] += (vectorX / magnitude) * 2 + 0.5;
            ys[i] += (vectorY / magnitude) * 2 + 0.5;
        }
        shape = new Polygon(xs, ys, count);
        return shape;
    }

    private Shape getExtendedShape(Road road, int extend) {
        Shape shape = null;
        int[] allApexes = road.getApexList();
        int count = allApexes.length / 2;
        int[] xs = new int[count];
        int[] ys = new int[count];
        double centerX = 0;
        double centerY = 0;
        for (int i = 0; i < count; ++i) {
            xs[i] = allApexes[i * 2];
            ys[i] = allApexes[i * 2 + 1];
            centerX += xs[i];
            centerY += ys[i];
        }
        centerX /= count;
        centerY /= count;
        for (int i = 0; i < count; ++i) {
            // 重心から頂点へのベクトル
            double vectorX = xs[i] - centerX;
            double vectorY = ys[i] - centerY;
            double magnitude = Math.sqrt(vectorX * vectorX + vectorY * vectorY); // ベクトルの大きさ
            // 重心から頂点への大きさ2のベクトルを頂点に足して四捨五入
            xs[i] += (vectorX / magnitude) * extend + 0.5;
            ys[i] += (vectorY / magnitude) * extend + 0.5;
        }
        shape = new Polygon(xs, ys, count);
        return shape;
    }

    private class HumanOnRoadComparator implements Comparator<Pair<StandardEntity, List<Human>>> {
        public int compare(Pair<StandardEntity, List<Human>> p1, Pair<StandardEntity, List<Human>> p2) {
            return p1.second().size() - p2.second().size();
        }
    }
}