package RIO_2019.extaction;


import adf.agent.action.Action;
import adf.agent.action.common.ActionMove;
import adf.agent.action.common.ActionRest;
import adf.agent.action.fire.ActionExtinguish;
import adf.agent.action.fire.ActionRefill;
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
import rescuecore2.misc.geometry.GeometryTools2D;
import rescuecore2.misc.geometry.Point2D;
import rescuecore2.standard.entities.*;
import rescuecore2.standard.entities.StandardEntityConstants.Fieryness;
import rescuecore2.worldmodel.EntityID;

import java.awt.Polygon;
import java.awt.Rectangle;
import java.util.*;


import static rescuecore2.standard.entities.StandardEntityURN.BUILDING;
import static rescuecore2.standard.entities.StandardEntityURN.HYDRANT;
import static rescuecore2.standard.entities.StandardEntityURN.REFUGE;

public class RIOActionFireFighting extends ExtAction {
    private PathPlanning pathPlanning;
    private Clustering clustering;

    private int maxExtinguishDistance;
    private int maxExtinguishPower;
    private int thresholdRest;
    private int kernelTime;
    private int refillCompleted;
    private int refillRequest;
    private boolean refillFlag;
    //private ArrayList<EntityID> previousPositions;
    private ArrayList<Integer> movedTime;

    private boolean stopped;
    private ArrayList<EntityID> unsearchedBuildingIDs;
    private int clusterIndex;
    private int changeClusterCycle;
    protected Random random;
    private ArrayList<Point2D> previousLocations;
    private ArrayList<Point2D> previousLocations2;
    private ArrayList<List<EntityID>> previousPaths;

    private boolean isRefuge;
    private boolean isRefill;

    private boolean isChangePos;
    private int changeCount;

    private EntityID target;

    public RIOActionFireFighting(AgentInfo agentInfo, WorldInfo worldInfo, ScenarioInfo scenarioInfo, ModuleManager moduleManager, DevelopData developData) {
        super(agentInfo, worldInfo, scenarioInfo, moduleManager, developData);
        this.maxExtinguishDistance = scenarioInfo.getFireExtinguishMaxDistance();
        this.maxExtinguishPower = scenarioInfo.getFireExtinguishMaxSum();
        this.thresholdRest = developData.getInteger("ActionFireFighting.rest", 100);
        int maxWater = scenarioInfo.getFireTankMaximum();
        this.refillCompleted = (maxWater / 10) * developData.getInteger("RIOActionFireFighting.refill.completed", 10);
        this.refillRequest = this.maxExtinguishPower * developData.getInteger("RIOActionFireFighting.refill.request", 1);
        this.refillFlag = false;
        //previousPositions = new ArrayList<>();
        unsearchedBuildingIDs = new ArrayList<>();
        movedTime = new ArrayList<>();
        this.changeClusterCycle = 5;
        this.clusterIndex = 0;
        this.random = new Random();
        this.stopped = false;
        this.previousLocations = new ArrayList<>();
        this.previousLocations2 = new ArrayList<>();
        this.previousPaths = new ArrayList<>();

        isRefuge = false;
        isRefill = false;

        this.isChangePos = false;
        this.changeCount = 1;

        this.target = null;

        switch (scenarioInfo.getMode()) {
            case PRECOMPUTATION_PHASE:
                this.pathPlanning = moduleManager.getModule("ActionFireFighting.PathPlanning", "RIO_2019.module.algorithm.AstarPathPlanning");
                this.clustering = moduleManager.getModule("ActionFireFighting.Clustering.Fire", "RIO_2019.module.algorithm.RioneKmeansPP");
                break;
            case PRECOMPUTED:
                this.pathPlanning = moduleManager.getModule("ActionFireFighting.PathPlanning", "RIO_2019.module.algorithm.AstarPathPlanning");
                this.clustering = moduleManager.getModule("ActionFireFighting.Clustering.Fire", "RIO_2019.module.algorithm.RioneKmeansPP");
                break;
            case NON_PRECOMPUTE:
                this.pathPlanning = moduleManager.getModule("ActionFireFighting.PathPlanning", "RIO_2019.module.algorithm.AstarPathPlanning");
                this.clustering = moduleManager.getModule("ActionFireFighting.Clustering.Fire", "RIO_2019.module.algorithm.RioneKmeansPP");
                break;
        }
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
            if (unsearchedBuildingIDs.contains(pID)) {
                unsearchedBuildingIDs.remove(pID);
            }
        }
        return this;
    }


    @Override
    public ExtAction setTarget(EntityID target) {
        this.target = null;
        if (target != null) {
            StandardEntity entity = this.worldInfo.getEntity(target);
            if (entity instanceof Building) {
                this.target = target;
            }
        }
        return this;
    }

    @Override
    public ExtAction calc() {
        this.result = null;
        FireBrigade agent = (FireBrigade) this.agentInfo.me();

		/*if (this.needRest(agent)){
			this.result = this.calcRefugeAction(agent, this.pathPlanning, this.target, false);
			if (this.result != null){
				return this;
			}
		}*/

        //this.refillFlag = this.needRefill(agent, this.refillFlag);
        //if (this.refillFlag){
        this.result = this.calcRefill();
        if (this.result != null) {
            return this;
        }
        //}

        this.result = this.calcGasStation();
        if (this.result != null) {
            return this;
        }

        this.result = this.calcBeforehand();
        if (this.result != null) {
            return this;
        }

        if (this.agentInfo.getTime() % 10 == 0 || this.isChangePos) {
            this.result = calcExtinguishPosition(agent, pathPlanning);
            if (this.result != null) {
                this.isChangePos = true;
                this.changeCount++;
                if (this.changeCount % 3 == 0) {
                    this.changeCount = 1;
                    this.isChangePos = false;
                }
                return this;
            }
        }

        this.result = this.calcExtinguish(agent, this.pathPlanning, this.target);
        if (this.result != null) {
            return this;
        }

        //targetがない＝探索が必要
        this.result = calcSearch();
        return this;

    }

    //予備消火
    private Action calcBeforehand() {
        FireBrigade fireBrigade = (FireBrigade) this.agentInfo.me();
        ArrayList<StandardEntity> allFB = new ArrayList<>(this.worldInfo.getEntitiesOfType(StandardEntityURN.FIRE_BRIGADE));//全ての「FB」
        ArrayList<StandardEntity> buildings = new ArrayList<>(this.worldInfo.getEntitiesOfType(BUILDING));//全ての「建物」
        ArrayList<StandardEntity> fireBuildings = new ArrayList<>();//全ての「燃えている建物」
        ArrayList<StandardEntity> BuildingsNearFire = new ArrayList<>();//全ての「燃えている建物付近の建物」

        for (StandardEntity entityB : buildings) {
            if (entityB instanceof Building) {
                Building building = (Building) entityB;
                //「建物」が燃えている場合
                if (building.isOnFire()) {
                    //全ての「燃えている建物」を特定
                    fireBuildings.add(entityB);
                }
            }
        }

        int fbNumber = 0;//「燃えている建物」付近の「FB」の数
        int fbAllDistance = 0;//「燃えている建物」から「FB」までの距離
        int fbDistance = 0;//「燃えている建物」から「自分」までの距離

        //「燃えている建物」がある場合
        if (!fireBuildings.isEmpty()) {
            for (StandardEntity entityFB : fireBuildings) {

                for (StandardEntity entityB : buildings) {
                    if (entityB instanceof Building) {
                        Building building = (Building) entityB;
                        //「建物」が燃えておらず、「燃えている建物」付近にいる場合
                        if (!building.isOnFire() && worldInfo.getDistance(entityB, entityFB) <= 10000) {
                            //全ての「燃えている建物付近の建物」を特定
                            BuildingsNearFire.add(entityB);
                        }
                    }
                }

                for (StandardEntity entityAFB : allFB) {
                    //「FB」が「燃えている建物」付近にいる場合
                    if (worldInfo.getDistance(entityAFB, entityFB) <= 20000) {
                        //「燃えている建物」付近にいる「FB」を数える
                        fbNumber++;
                        if (worldInfo.getDistance(entityAFB, entityFB) > fbAllDistance) {
                            fbAllDistance = worldInfo.getDistance(entityAFB, entityFB);
                        }
                        fbDistance = worldInfo.getDistance(fireBrigade, entityFB);
                    }
                }

            }
        }

        StandardEntity Target = null;

        int bnfDistance = maxExtinguishDistance;//「燃えている建物付近の建物」から「FB」までの距離

        //「燃えている建物」付近にいる多くの「FB」がおり（とりあえず自分を除いて3人以上）、
        //その中でも自分が「燃えている建物」から最も遠い場合
        if (fbNumber >= 3 && fbDistance == fbAllDistance) {
            for (StandardEntity entityBNF : BuildingsNearFire) {
                if (worldInfo.getDistance(fireBrigade, entityBNF) < bnfDistance) {
                    bnfDistance = worldInfo.getDistance(fireBrigade, entityBNF);
                    Target = entityBNF;
                }
            }
        }

        //燃えている建物がなければ何もしない
        if (Target == null) {
            return null;
        }

        //エージェントが「燃えている建物付近の建物」付近にいる場合
        if (worldInfo.getDistance(fireBrigade, Target) <= maxExtinguishDistance) {
            return new ActionExtinguish(Target.getID(), this.maxExtinguishPower);
        }
        //いない場合
        else {
            //「燃えている建物付近の建物」への道を特定
            List<EntityID> path = null;
            pathPlanning.setFrom(fireBrigade.getPosition());
            pathPlanning.setDestination(Target.getID());
            path = pathPlanning.calc().getResult();
            //「燃えている建物付近の建物」への道がある場合
            if (path != null && !path.isEmpty()) {
                //「燃えている建物付近の建物」に移動
                return new ActionMove(path, ((Building) Target).getX(), ((Building) Target).getY());
            }
        }

        return null;
    }

    //燃焼度を無視
    private Action calcGasStation() {
        FireBrigade fireBrigade = (FireBrigade) this.agentInfo.me();
        ArrayList<StandardEntity> buildings = new ArrayList<>(this.worldInfo.getEntitiesOfType(BUILDING));//全ての「建物」
        ArrayList<StandardEntity> gasStations = new ArrayList<>(this.worldInfo.getEntitiesOfType(StandardEntityURN.GAS_STATION));//全ての「ガスステーション」
        ArrayList<StandardEntity> nearBuildings = new ArrayList<>();//全ての「ガスステーション付近の建物」
        ArrayList<StandardEntity> nearFireBuildings = new ArrayList<>();//全ての「ガスステーション付近で燃えている建物」

        for (StandardEntity entityB : buildings) {
            for (StandardEntity entityG : gasStations) {
                if (entityG instanceof Building) {
                    Building building = (Building) entityG;
                    //「建物」が「ガスステーション」付近にあり、「ガスステーション」が燃えていない場合
                    if (worldInfo.getDistance(entityB, entityG) <= 20000 && building.getFierynessEnum() == Fieryness.UNBURNT) {
                        //全ての「ガスステーション付近の建物」を特定
                        nearBuildings.add(entityB);
                    }
                }
            }
        }

        for (StandardEntity entityNB : nearBuildings) {
            if (entityNB instanceof Building) {
                Building building = (Building) entityNB;
                //「ガスステーション付近の建物」が燃えている場合
                if (building.isOnFire()) {
                    //全ての「ガスステーション付近で燃えている建物」を特定
                    nearFireBuildings.add(entityNB);
                }
            }
        }

        StandardEntity Target = null;
        //「ガスステーション付近で燃えている建物」がある場合
        if (!nearFireBuildings.isEmpty()) {
            for (StandardEntity entityNF : nearFireBuildings) {
                //エージェントが「ガスステーション付近で燃えている建物」付近にいる場合
                if (worldInfo.getDistance(fireBrigade, entityNF) <= maxExtinguishDistance) {
                    return new ActionExtinguish(entityNF.getID(), this.maxExtinguishPower);
                }
                //いない場合
                else {
                    Target = entityNF;
                }
            }
        }

        if (Target != null) {
            //「ガスステーション付近で燃えている建物」への道を特定
            List<EntityID> path = null;
            pathPlanning.setFrom(fireBrigade.getPosition());
            pathPlanning.setDestination(Target.getID());
            path = pathPlanning.calc().getResult();
            //「ガスステーション付近で燃えている建物」への道がある場合
            if (path != null && !path.isEmpty()) {
                //「ガスステーション付近で燃えている建物」に移動
                return new ActionMove(path, ((Building) Target).getX(), ((Building) Target).getY());
            }
        }

        return null;
    }

    //西田孝典
    private Action calcSearch() {// パスの生成、つっかえたら作りなおす
        if (agentInfo.getTime() < scenarioInfo.getKernelAgentsIgnoreuntil()) {
            return null;
        }
        return getSearchAction(pathPlanning, this.agentInfo.getPosition(), this.unsearchedBuildingIDs);

    }

    private void reset() {
        this.unsearchedBuildingIDs.clear();
        this.previousPaths.clear();
        this.previousLocations.clear();

        if ((this.agentInfo.getTime() != 0 && (this.agentInfo.getTime() % this.changeClusterCycle) == 0) || stopped) {
            this.stopped = false;
            this.clusterIndex = random.nextInt(clustering.getClusterNumber());
            this.changeClusterCycle = random.nextInt(31) + 30;

        }

        Collection<StandardEntity> clusterEntities = new ArrayList<>();
        if (clustering != null) {
            //System.out.println("inin");
            clusterEntities.addAll(this.clustering.getClusterEntities(clusterIndex));
        }


        if (clusterEntities != null && clusterEntities.size() > 0) {
            for (StandardEntity entity : clusterEntities) {
                if (entity instanceof Building && entity.getStandardURN() != REFUGE) {
                    this.unsearchedBuildingIDs.add(entity.getID());
                }
            }
        } else {
            this.unsearchedBuildingIDs.addAll(this.worldInfo.getEntityIDsOfType(BUILDING));
        }
    }

    private Action calcExtinguish(FireBrigade agent, PathPlanning pathPlanning, EntityID target) {
        EntityID agentPosition = agent.getPosition();
        StandardEntity positionEntity = Objects.requireNonNull(this.worldInfo.getPosition(agent));
        if (StandardEntityURN.REFUGE == positionEntity.getStandardURN()) {int Maxwater = scenarioInfo.getFireTankMaximum();
            int water = agent.getWater();
            if (water < Maxwater * 0.9) {
                return new ActionRefill();
            } else {
                return getMoveAction(pathPlanning, agent.getPosition(), target);
            }
        }

        List<StandardEntity> neighbourBuilding = new ArrayList<>();
        StandardEntity entity = this.worldInfo.getEntity(target);
        if (entity instanceof Building) {
            if (calculateWallDistance(entity) < maxExtinguishDistance) {
                neighbourBuilding.add(entity);
            }
        }

        if (neighbourBuilding.size() > 0) {
            neighbourBuilding.sort(new DistanceSorter(this.worldInfo, agent));
            for (EntityID entityID : this.worldInfo.getChanged().getChangedEntities()) {
                if (entityID.equals(neighbourBuilding.get(0).getID())) {
                    return new ActionExtinguish(neighbourBuilding.get(0).getID(), this.maxExtinguishPower);
                }
            }
        }
        return this.getMoveAction(pathPlanning, agentPosition, target);
    }

    private int calculateWallDistance(StandardEntity se) {
        int minDist = Integer.MAX_VALUE;
        if (se instanceof Building) {
            List<Edge> edges = ((Building) se).getEdges();
            for (Edge edge : edges) {
                Point2D closestPoint = GeometryTools2D.getClosestPointOnSegment(edge.getLine(), new Point2D(agentInfo.getX(), agentInfo.getY()));
                int dist = (int) Math.hypot(closestPoint.getX() - agentInfo.getX(), closestPoint.getY() - agentInfo.getY());
                if (dist < minDist) {
                    minDist = dist;
                }
            }
        }
        return minDist;
    }


    //nishida
    private Action getMoveAction(PathPlanning pathPlanning, EntityID from, EntityID target) {
        pathPlanning.setFrom(from);
        pathPlanning.setDestination(target);
        List<EntityID> path = pathPlanning.calc().getResult();
        //if (path == null) System.out.println("パスプラがヌル@actionFireFighting");
        previousPaths.add(path);

        if (previousPaths.size() < 2 || !isStopped(previousPaths.get(0), previousPaths.get(1))) {//止まってるかどうか判定
            if (path != null && path.size() > 0) {
                StandardEntity entity = this.worldInfo.getEntity(path.get(path.size() - 1));
                if (entity instanceof Building) {
                    if (entity.getStandardURN() != StandardEntityURN.REFUGE) {
                        path.remove(path.size() - 1);
                    }
                }
                movedTime.add(agentInfo.getTime());//動いた時のTimeを記録
                return new ActionMove(path);
            }
        }
        return null;
    }

    //西田孝典
    private Action getSearchAction(PathPlanning pathPlanning, EntityID from, Collection<EntityID> targets) {
        pathPlanning.setFrom(from);
        pathPlanning.setDestination(targets);
        List<EntityID> path = pathPlanning.calc().getResult();
        previousPaths.add(path);
        //System.out.println(path);
        if (previousPaths.size() < 2 || !isStopped(previousPaths.get(0), previousPaths.get(1))) {//止まってるかどうか判定
            if (path != null && path.size() > 0) {
                StandardEntity entity = this.worldInfo.getEntity(path.get(path.size() - 1));
                if (entity instanceof Building) {
                    if (entity.getStandardURN() != StandardEntityURN.REFUGE) {
                        path.remove(path.size() - 1);
                    }
                }
                movedTime.add(agentInfo.getTime());//動いた時のTimeを記録
                return new ActionMove(path);
            }
            return null;
        }
        this.stopped = true;
        reset();
        return null;
    }

	/*
	//nishida
	private boolean isStopped(EntityID from) {
		previousPositions.add(from);//移動するときの場所を記録(0が現在地)
		if(previousPositions.size()>2){
			if(movedTime.contains(agentInfo.getTime()-1)
					&& (movedTime.contains(agentInfo.getTime()-2))
					&& (previousPositions.get(0).equals(previousPositions.get(1)))
					&& (previousPositions.get(0).equals(previousPositions.get(2)))){//2サイクル連続moveしてて，2サイクル連続同じ場所にいたら
				return true;//止まっている
			}
		}
		return false;
	}
	 */

    private boolean isStopped(List<EntityID> path1, List<EntityID> path2) {
        Human agent = (Human) this.agentInfo.me();
        previousLocations.add(new Point2D(agent.getX(), agent.getY()));//移動するときの場所を記録(0が現在地)
        if (path1 == null || path2 == null) {
            return false;
            //System.out.println("flag,false1");
        }

        if (path1.size() != path2.size()) {
            //System.out.println("flag,false2");
            return false;
        } else {
            for (int i = 0; i < path1.size(); i++) {
                EntityID id1 = path1.get(i);
                EntityID id2 = path2.get(i);
                if (!id1.equals(id2)) {
                    return false;
                    //System.out.println("flag,false3");
                }
            }
        }

        if (previousLocations.size() > 2) {
            //System.out.println("flag,true");
            return withinRange(previousLocations.get(0), previousLocations.get(1), previousLocations.get(2), 30000);
        }
        //System.out.println("isStopped,false");
        return false;

    }

    private boolean isStopped2(List<EntityID> path1, List<EntityID> path2) {
        Human agent = (Human) this.agentInfo.me();
        previousLocations2.add(new Point2D(agent.getX(), agent.getY()));//移動するときの場所を記録(0が現在地)
        if (path1 == null || path2 == null) {
            return false;
            //System.out.println("flag,false1");
        }

        if (path1.size() != path2.size()) {
            //System.out.println("flag,false2");
            return false;
        } else {
            for (int i = 0; i < path1.size(); i++) {
                EntityID id1 = path1.get(i);
                EntityID id2 = path2.get(i);
                if (!id1.equals(id2)) {
                    return false;
                    //System.out.println("flag,false3");
                }
            }
        }

        List<StandardEntity> neighbourBuilding = new ArrayList<>();
        StandardEntity entity = this.worldInfo.getEntity(target);
        if (entity instanceof Building) {
            if (calculateWallDistance(entity) < maxExtinguishDistance) {
                neighbourBuilding.add(entity);
            }
        }

        if (neighbourBuilding.size() > 0) {
            neighbourBuilding.sort(new DistanceSorter(this.worldInfo, agent));
            for (EntityID entityID : this.worldInfo.getChanged().getChangedEntities()) {
                if (entityID.equals(neighbourBuilding.get(0).getID())) {
                    return false;
                }
            }
        }


        //視界内の燃えている建物ではない時
        if (previousLocations2.size() > 2) {
            //System.out.println("flag,true");
            return withinRange(previousLocations2.get(0), previousLocations2.get(1), previousLocations2.get(2), 5000);
        }
        //System.out.println("isStopped,false");
        return false;

    }

    private boolean withinRange(Point2D position1, Point2D position2, Point2D position3, int range) {

        double dist1 = GeometryTools2D.getDistance(position1, position2);
        double dist2 = GeometryTools2D.getDistance(position1, position3);
        if (dist1 < range && dist2 < range) {
            //System.out.println("isStopped,true");
            return true;
        }
        //System.out.println("isStopped,false");
        return false;
    }

    private EntityID reCalclateTarget(EntityID preTarget) {
        EntityID ref;
        ref = calcTargetInWorld(preTarget);
        if (ref == null || ref.equals(preTarget))
            return null;

        return ref;


    }

    private EntityID calcTargetInWorld(EntityID preTarget) {
        Collection<StandardEntity> ses = new ArrayList<StandardEntity>();
        ses.addAll(this.worldInfo.getEntitiesOfType(
                StandardEntityURN.BUILDING,
                StandardEntityURN.GAS_STATION,
                StandardEntityURN.AMBULANCE_CENTRE,
                StandardEntityURN.FIRE_STATION,
                StandardEntityURN.POLICE_OFFICE
        ));
        List<Building> targets = new ArrayList<Building>();
        targets.addAll(filterFiery(ses));
        List<Building> filteredTargets = new ArrayList<Building>();
        filteredTargets.addAll(targets);

        if (targets == null || targets.isEmpty()) {
            return null;
        } else {
            for (Building bu : targets) {
                if (bu.getID().equals(preTarget))
                    filteredTargets.remove(bu);
            }
            if (filteredTargets != null && !filteredTargets.isEmpty()) {
                Collections.sort(filteredTargets, new DistanceSorter(worldInfo, agentInfo.me()));
                Building selectedBuilding = filteredTargets.get(0);
                return selectedBuilding.getID();
            }
            return null;
        }
    }

    private List<Building> filterFiery(Collection<? extends StandardEntity> input) {
        ArrayList<Building> fireBuildings = new ArrayList<>();
        for (StandardEntity entity : input) {
            if (entity instanceof Building && ((Building) entity).isOnFire()) {
                fireBuildings.add((Building) entity);
            }
        }
        return filterFieryness(fireBuildings);
    }

    //nishida
    //燃焼度の低いものを優先
    private List<Building> filterFieryness(Collection<? extends StandardEntity> input) {
        ArrayList<Building> fireBuildings = new ArrayList<>();
        for (StandardEntity entity : input) {
            if (entity instanceof Building
                    && ((Building) entity).isOnFire()
                    && ((Building) entity).getFierynessEnum() == Fieryness.HEATING) {
                fireBuildings.add((Building) entity);
            }
        }
        if (!fireBuildings.isEmpty())
            return fireBuildings;

        for (StandardEntity entity : input) {
            if (entity instanceof Building
                    && ((Building) entity).isOnFire()
                    && ((Building) entity).getFierynessEnum() == Fieryness.BURNING) {
                fireBuildings.add((Building) entity);
            }
        }
        if (!fireBuildings.isEmpty())
            return fireBuildings;

        for (StandardEntity entity : input) {
            if (entity instanceof Building
                    && ((Building) entity).isOnFire()
                    && ((Building) entity).getFierynessEnum() == Fieryness.INFERNO) {
                fireBuildings.add((Building) entity);
            }
        }
        return fireBuildings;
    }


    private boolean needRefill(FireBrigade agent, boolean refillFlag) {
        if (refillFlag) {
            StandardEntityURN positionURN = Objects.requireNonNull(this.worldInfo.getPosition(agent)).getStandardURN();
            return !(positionURN == REFUGE || positionURN == HYDRANT) || agent.getWater() < this.refillCompleted;
        }
        return agent.getWater() <= this.refillRequest;
    }

    private boolean needRest(Human agent) {
        int hp = agent.getHP();
        int damage = agent.getDamage();
        if (hp == 0 || damage == 0) {
            return false;
        }
        int activeTime = (hp / damage) + ((hp % damage) != 0 ? 1 : 0);
        if (this.kernelTime == -1) {
            try {
                this.kernelTime = this.scenarioInfo.getKernelTimesteps();
            } catch (NoSuchConfigOptionException e) {
                this.kernelTime = -1;
            }
        }
        return damage >= this.thresholdRest || (activeTime + this.agentInfo.getTime()) < this.kernelTime;
    }


    //追林拓光，西田孝典
    private Action calcRefill() {
        FireBrigade fireBrigade = (FireBrigade) this.agentInfo.me();
        EntityID agentPosition = fireBrigade.getPosition();
        int water = fireBrigade.getWater();//水の量
        StandardEntityURN positionURN = this.worldInfo.getPosition(fireBrigade).getStandardURN();
        int maxWater = scenarioInfo.getFireTankMaximum();
        int Damage = fireBrigade.getDamage();
        int maxExtinguishPower = scenarioInfo.getFireExtinguishMaxSum();
        //西田
        //ダメージが0より上なら，リフジに強制移動
        if (Damage > 0) {
            isRefuge = true;
        }
        /*
         * リフジ,ハイドラントにいるとき
         * 満タンかつ体力が9000より上なら，給水しない
         */
        if (positionURN.equals(StandardEntityURN.REFUGE)) {
            isRefuge = false;
            if (water < maxWater * 0.9) {
                return new ActionRefill();
            } else {
                return null;
            }
        }

        if (positionURN.equals(StandardEntityURN.HYDRANT) && isRefill) {
            isRefuge = false;
            if (water < maxWater * 0.9 && isHydrant()) {
                return new ActionRefill();
            } else {
                isRefill = false;
                return null;
            }


        }
        //リフジ強制移動じゃないとき，入る
        if (!isRefuge) {

            /*
             * タンクの水が1割より多いとき，給水しない
             *
             * タンクの水が1割以下，
             * かつ視界内のFBが3人以上のとき，
             * その中で一番タンクの水が少ないFBが給水する
             *
             * タンクの水が1割以下，
             * かつ視界内のFBが2人以下のとき，
             * タンクの水が0になった時に給水する
             */
            if (water > maxWater / 10) {
                return null;
            } else {
                FireBrigade minFB = (FireBrigade) agentInfo.me();
                ArrayList<FireBrigade> visibleFireBrigades = new ArrayList<>();

                for (EntityID id : worldInfo.getChanged().getChangedEntities()) {
                    StandardEntity standardEntity = worldInfo.getEntity(id);
                    if (standardEntity instanceof FireBrigade) {
                        FireBrigade fb = (FireBrigade) standardEntity;
                        visibleFireBrigades.add(fb);
                    }
                }
                if (visibleFireBrigades.size() > 2) {
                    for (FireBrigade fb : visibleFireBrigades) {
                        if (fb.isWaterDefined() && fb.getWater() < minFB.getWater()) {
                            minFB = fb;
                        }
                    }
                    if (((FireBrigade) agentInfo.me()).equals(minFB)) {
                        return calcRefillPosition(fireBrigade, agentPosition);
                    }
                }
            }
            //タンクの水が0になったとき，同様に判断して，消火栓に移動，若しくはリフジに強制移動
            if (water == 0) {
                this.result = calcRefillPosition(fireBrigade, agentPosition);
                return this.result;
            }
        } else {
            //リフジに移動
            List<EntityID> path = null;
            pathPlanning.setFrom(fireBrigade.getPosition());
            pathPlanning.setDestination(this.worldInfo.getEntityIDsOfType(StandardEntityURN.REFUGE));
            path = pathPlanning.calc().getResult();
            if (path != null && !path.isEmpty()) {
                this.result = this.getMoveAction(pathPlanning, agentPosition, path.get(path.size() - 1));
                return this.result;
            }
        }
        return null;
    }


    //追林
    private Action calcRefillPosition(FireBrigade fireBrigade, EntityID agentPosition) {
        HashMap<Hydrant, Integer> count = new HashMap<Hydrant, Integer>();

        //それぞれの消火栓に何人いるか
        for (StandardEntity fb : this.worldInfo.getEntitiesOfType(StandardEntityURN.FIRE_BRIGADE)) {
            StandardEntity se = this.worldInfo.getPosition((Human) fb);
            if (se instanceof Hydrant) {
                Hydrant hy = (Hydrant) se;
                if (count.containsKey(hy)) {
                    count.put(hy, count.get(hy) + 1);//すでに他のエージェントいたら，値を+1
                } else {
                    count.put(hy, 1);
                }
            }
        }
        List<EntityID> path = null;
        int tmp = Integer.MAX_VALUE; //テキトー
        StandardEntity Target = null;

        //一番近い消火栓
        for (StandardEntity hyd : this.worldInfo.getEntitiesOfType(StandardEntityURN.HYDRANT)) {
            int target = worldInfo.getDistance(fireBrigade, hyd);
            if (target < tmp) {
                tmp = target;
                Target = hyd;
            }
        }
        /*
         * 一番近い消火栓に誰もいなければ，消火栓かリフジ効率良い方へ移動
         *
         * 一番近い消火栓に誰かいれば，リフジに強制移動
         */
        if (Target != null && count.get(Target) == null) {
            this.result = calcEfficientRefill(fireBrigade, pathPlanning, agentPosition, Target);
            return this.result;
        } else {
            isRefuge = true;
            pathPlanning.setFrom(fireBrigade.getPosition());
            pathPlanning.setDestination(this.worldInfo.getEntityIDsOfType(StandardEntityURN.REFUGE));
            path = pathPlanning.calc().getResult();
            if (path != null && !path.isEmpty()) {
                this.result = this.getMoveAction(pathPlanning, agentPosition, path.get(path.size() - 1));
                return this.result;
            }
        }
        return null;
    }

    //追林
    private boolean isHydrant() {
        HashMap<Hydrant, Integer> count = new HashMap<Hydrant, Integer>();
        FireBrigade fireBrigade = (FireBrigade) this.agentInfo.me();

        //それぞれの消火栓に何人いるか
        for (StandardEntity fb : this.worldInfo.getEntitiesOfType(StandardEntityURN.FIRE_BRIGADE)) {
            StandardEntity se = this.worldInfo.getPosition((Human) fb);
            if (se instanceof Hydrant) {
                Hydrant hy = (Hydrant) se;
                if (count.containsKey(hy)) {
                    count.put(hy, count.get(hy) + 1);//すでに他のエージェントいたら，値を+1
                } else {
                    count.put(hy, 1);
                }
            }
        }
        List<EntityID> path = null;
        int tmp = Integer.MAX_VALUE; //テキトー
        StandardEntity Target = null;

        //一番近い消火栓
        for (StandardEntity hyd : this.worldInfo.getEntitiesOfType(StandardEntityURN.HYDRANT)) {
            int target = worldInfo.getDistance(fireBrigade, hyd);
            if (target < tmp) {
                tmp = target;
                Target = hyd;
            }
        }
        /*
         * 一番近い消火栓に誰もいなければ，消火栓へ移動
         *
         * 一番近い消火栓に誰かいれば，リフジに強制移動
         */
        if (Target != null && count.get(Target) == null) {
            return true;
        }
        return false;
    }


    private Action calcEfficientRefill(FireBrigade fireBrigade, PathPlanning pathPlanning, EntityID target, StandardEntity hydrant) {
        double hydrantRate = scenarioInfo.getFireTankRefillHydrantRate();//hydrantの補給効率
        double refugeRate = scenarioInfo.getFireTankRefillRate();//refugeの補給効率
        double firebrigadecapa = scenarioInfo.getFireTankMaximum();//fbのタンクの最大容量


        pathPlanning.setFrom(fireBrigade.getPosition());
        pathPlanning.setDestination(this.worldInfo.getEntityIDsOfType(StandardEntityURN.REFUGE));
        List<EntityID> refugePath = pathPlanning.calc().getResult();

        pathPlanning.setFrom(fireBrigade.getPosition());
        pathPlanning.setDestination(hydrant.getID());
        List<EntityID> hydrantPath = pathPlanning.calc().getResult();

        if (refugePath == null || refugePath.isEmpty()) {
            if (hydrantPath != null && !hydrantPath.isEmpty()) {
                return getMoveAction(pathPlanning, fireBrigade.getPosition(), hydrant.getID());
            }
        }
        if (hydrantPath == null || hydrantPath.isEmpty()) {
            if (refugePath != null && !refugePath.isEmpty()) {
                isRefuge = true;
                return getMoveAction(pathPlanning, fireBrigade.getPosition(), refugePath.get(refugePath.size() - 1));
            }
        }
        if (refugePath == null && hydrantPath == null || refugePath.isEmpty() && hydrantPath.isEmpty()) {
            return null;
        }


        if ((firebrigadecapa / hydrantRate - firebrigadecapa / refugeRate) * 0.2 >= 2 * (refugePath.size() / 30000 - hydrantPath.size() / 30000)) {
            if (hydrantPath != null && !hydrantPath.isEmpty()) {
                isRefill = true;
                return getMoveAction(pathPlanning, fireBrigade.getPosition(), hydrant.getID());
            }
        } else {
            if (refugePath != null && !refugePath.isEmpty()) {
                isRefuge = true;
                return getMoveAction(pathPlanning, fireBrigade.getPosition(), refugePath.get(refugePath.size() - 1));
            }
        }
        return null;
    }


    //ExtinguishPosition
    //西田孝典
    private Action calcExtinguishPosition(FireBrigade agent, PathPlanning pathPlanning) {
        ArrayList<Building> fireBuildings = new ArrayList<>();//燃えている建物の集まり
        ArrayList<Building> infernoBuildings = new ArrayList<>();//inferno状態の建物の集まり
        ArrayList<Building> burningBuildings = new ArrayList<>();//burning状態の建物の集まり
        ArrayList<Building> heatingBuildings = new ArrayList<>(); //heating状態の建物の集まり
        ArrayList<Building> visibleFireBuildings = new ArrayList<>();//燃えている建物の集まり
        ArrayList<Building> visibleInfernoBuildings = new ArrayList<>();//視界内のinferno状態の建物の集まり
        ArrayList<Building> visibleBurningBuildings = new ArrayList<>();//視界内のburning状態の建物の集まり
        ArrayList<Building> visibleHeatingBuildings = new ArrayList<>(); //視界内のheating状態の建物の集まり
        List<EntityID> fireBrigadeIDs = new ArrayList<>();
        List<Integer> fireBrigadeIDsvalue = new ArrayList<>();
        int i = 0;
        int j = 0;
        int vertex1X;
        int vertex2X;
        int vertex1Y;
        int vertex3Y;


        for (StandardEntity entity : worldInfo.getEntitiesOfType(StandardEntityURN.BUILDING)) {
            if (entity instanceof Building) {
                Building building = (Building) entity;
                if (building.isOnFire()) {
                    fireBuildings.add(building);
                }
            }
        }

        ArrayList<FireBrigade> visibleFireBrigades = new ArrayList<>();

        for (EntityID id : worldInfo.getChanged().getChangedEntities()) {
            StandardEntity standardEntity = worldInfo.getEntity(id);
            if (standardEntity instanceof FireBrigade) {
                FireBrigade fireBrigade = (FireBrigade) standardEntity;
                visibleFireBrigades.add(fireBrigade);
            }
        }

        /*
         * 視界内の建物の中で燃えている建物をvisibleFireBuildingsにいれる．
         * また、その中でINFERNO，BURNING，HEATING状態の建物を
         * それぞれ、visibleInfernoBuildings，visibleBurningBuildings，visibleHeatingBuildingsに入れる
         */
        for (EntityID id : worldInfo.getChanged().getChangedEntities()) {
            StandardEntity standardEntity = worldInfo.getEntity(id);
            if (standardEntity instanceof Building) {
                Building building = (Building) standardEntity;
                if (building.isOnFire()) {
                    visibleFireBuildings.add(building);
                    if (building.getFierynessEnum() == Fieryness.INFERNO) {
                        visibleInfernoBuildings.add(building);
                    } else if (building.getFierynessEnum() == Fieryness.BURNING) {
                        visibleBurningBuildings.add(building);
                    } else if (building.getFierynessEnum() == Fieryness.HEATING) {
                        visibleHeatingBuildings.add(building);
                    }
                }
            }
        }
        if (!fireBuildings.isEmpty()) {
            for (Building firebuilding : fireBuildings) {
                if (firebuilding.getFierynessEnum() == Fieryness.INFERNO) {
                    infernoBuildings.add(firebuilding);
                } else if (firebuilding.getFierynessEnum() == Fieryness.BURNING) {
                    burningBuildings.add(firebuilding);
                } else if (firebuilding.getFierynessEnum() == Fieryness.HEATING) {
                    heatingBuildings.add(firebuilding);
                }
            }

            ///////////サーチ条件////////////////////////////////////////////////////////////////////////////////////////

			/*if(visibleHeatingBuildings.isEmpty()&&heatingBuildings.isEmpty()) {
				return calcSearch();
			}*/

            ////////////移動条件//////////////////////////////////////////////////////////////////////////////////////////////
            if (visibleFireBrigades.size() > 3
                    || (visibleBurningBuildings.isEmpty() && visibleHeatingBuildings.isEmpty())
                    || !(visibleBurningBuildings.isEmpty() &&
                    visibleHeatingBuildings.isEmpty() &&
                    burningBuildings.isEmpty() &&
                    heatingBuildings.isEmpty() &&
                    !(visibleInfernoBuildings.isEmpty()))) {

                //fireBuildingsの多角形を作る
                //その多角形を覆う長方形を作る
                //その長方形の各頂点の座標を取り
                //fireBuildingsの中心と頂点との距離がもっとも大きい物を半径rとする
                if (!fireBuildings.isEmpty()) {
                    int[] fireBuildingsx = new int[burningBuildings.size() + heatingBuildings.size()];//fireBuildingsのx座標の集まり
                    int[] fireBuildingsy = new int[burningBuildings.size() + heatingBuildings.size()];//fireBuildingsのｙ座標の集まり
                    for (Building building : burningBuildings) {
                        fireBuildingsx[i] = building.getX();
                        fireBuildingsy[i] = building.getY();
                        i++;
                        j = i;
                    }
                    for (Building building : heatingBuildings) {
                        fireBuildingsx[i] = building.getX();
                        fireBuildingsy[j] = building.getY();
                        j++;
                    }

                    Polygon fireBuildingsPolygon = new Polygon(fireBuildingsx, fireBuildingsy, fireBuildingsx.length);//fireBuildingsの多角形
                    Rectangle fireBuildingsRange = fireBuildingsPolygon.getBounds();//多角形fireBuildingsPolygonを覆う長方形
                    vertex1X = fireBuildingsRange.x;//長方形fireBuildingsRangeの左上の頂点のx座標
                    vertex1Y = fireBuildingsRange.y;//長方形fireBuildingsRangeの左上の頂点のy座標
                    vertex2X = vertex1X + fireBuildingsRange.width;//長方形fireBuildingsRangeの右上の頂点のx座標
                    vertex3Y = vertex1Y - fireBuildingsRange.height;//長方形fireBuildingsRangeの左下の頂点のy座標


                    Building topBuilding = fireBuildings.get(0);
                    Building bottomBuilding = fireBuildings.get(0);
                    Building leftBuilding = fireBuildings.get(0);
                    Building rightBuilding = fireBuildings.get(0);
                    for (Building building : fireBuildings) {
                        if (Math.abs(building.getY() - vertex1Y) < Math.abs(topBuilding.getY() - vertex1Y)) {
                            topBuilding = building;
                        }
                        if (Math.abs(building.getY() - vertex3Y) < Math.abs(bottomBuilding.getY() - vertex3Y)) {
                            bottomBuilding = building;
                        }
                        if (Math.abs(building.getX() - vertex2X) < Math.abs(rightBuilding.getX() - vertex2X)) {
                            rightBuilding = building;
                        }
                        if (Math.abs(building.getX() - vertex1X) < Math.abs(leftBuilding.getX() - vertex1X)) {
                            leftBuilding = building;
                        }
                    }

                    //ArrayList<Priority> priority = new ArrayList<>();
                    ArrayList<EntityID> topNeighbor1 = new ArrayList<>();
                    ArrayList<EntityID> topNeighbor2 = new ArrayList<>();

                    for (EntityID entityID : topBuilding.getNeighbours()) {
                        if (!topNeighbor1.contains(entityID)) {
                            topNeighbor1.add(entityID);
                        }
                    }

                    for (EntityID entityID : topNeighbor1) {
                        StandardEntity entity = this.worldInfo.getEntity(entityID);
                        if (entity instanceof Road) {
                            Road road = (Road) entity;
                            for (EntityID id : road.getNeighbours()) {
                                if (!topNeighbor2.contains(id)) {
                                    topNeighbor2.add(id);
                                }
                            }
                        }

                    }


                    ArrayList<EntityID> bottomNeighbor1 = new ArrayList<>();
                    ArrayList<EntityID> bottomNeighbor2 = new ArrayList<>();
                    ArrayList<EntityID> bottomNeighborBuildings = new ArrayList<>();

                    for (EntityID entityID : bottomBuilding.getNeighbours()) {
                        if (!bottomNeighbor1.contains(entityID)) {
                            bottomNeighbor1.add(entityID);
                        }
                    }

                    for (EntityID entityID : bottomNeighbor1) {
                        StandardEntity entity = this.worldInfo.getEntity(entityID);
                        if (entity instanceof Road) {
                            Road road = (Road) entity;
                            for (EntityID id : road.getNeighbours()) {
                                if (!bottomNeighbor2.contains(id)) {
                                    bottomNeighbor2.add(id);
                                }
                            }
                        }
                    }


                    ArrayList<EntityID> rightNeighbor1 = new ArrayList<>();
                    ArrayList<EntityID> rightNeighbor2 = new ArrayList<>();
                    ArrayList<EntityID> rightNeighborBuildings = new ArrayList<>();

                    for (EntityID entityID : rightBuilding.getNeighbours()) {
                        if (!rightNeighbor1.contains(entityID)) {
                            rightNeighbor1.add(entityID);
                        }
                    }

                    for (EntityID entityID : rightNeighbor1) {
                        StandardEntity entity = this.worldInfo.getEntity(entityID);
                        if (entity instanceof Road) {
                            Road road = (Road) entity;
                            for (EntityID id : road.getNeighbours()) {
                                if (!rightNeighbor2.contains(id)) {
                                    rightNeighbor2.add(id);
                                }
                            }
                        }
                    }

                    ArrayList<EntityID> leftNeighbor1 = new ArrayList<>();
                    ArrayList<EntityID> leftNeighbor2 = new ArrayList<>();
                    ArrayList<EntityID> leftNeighborBuildings = new ArrayList<>();

                    for (EntityID entityID : leftBuilding.getNeighbours()) {
                        if (!leftNeighbor1.contains(entityID)) {
                            leftNeighbor1.add(entityID);
                        }
                    }

                    for (EntityID entityID : leftNeighbor1) {
                        StandardEntity entity = this.worldInfo.getEntity(entityID);
                        if (entity instanceof Road) {
                            Road road = (Road) entity;
                            for (EntityID id : road.getNeighbours()) {
                                if (!leftNeighbor2.contains(id)) {
                                    leftNeighbor2.add(id);
                                }
                            }
                        }
                    }


                    for (StandardEntity entity : worldInfo.getEntitiesOfType(StandardEntityURN.FIRE_BRIGADE)) {
                        if (entity instanceof FireBrigade) {
                            FireBrigade fireBrigade = (FireBrigade) entity;
                            fireBrigadeIDs.add(fireBrigade.getID());
                        }
                    }


                    if (!fireBrigadeIDs.isEmpty()) {
                        for (EntityID id : fireBrigadeIDs) {
                            fireBrigadeIDsvalue.add(id.getValue());
                        }
                        Collections.sort(fireBrigadeIDsvalue);
                    }


                    int number = 0;
                    if (!fireBrigadeIDsvalue.isEmpty()) {
                        for (i = 0; i < fireBrigadeIDsvalue.size(); i++) {
                            if (agentInfo.me().getID().getValue() == fireBrigadeIDsvalue.get(i)) {
                                if (i % 4 == 0) {
                                    number = 0;
                                } else if (i % 4 == 1) {
                                    number = 1;
                                } else if (i % 4 == 2) {
                                    number = 2;
                                } else if (i % 4 == 3) {
                                    number = 3;
                                }
                            }
                        }
                    }
                    if (number == 0 && !(topNeighbor2.isEmpty())) {

                        //System.out.println(agentInfo.me().getID().getValue() + "getPassableMoveAction");
                        return getMoveAction(pathPlanning, agent.getPosition(), topNeighbor2.get(0));
                    }
                    if (number == 1 && !(bottomNeighborBuildings.isEmpty())) {

                        //System.out.println(agentInfo.me().getID().getValue() + "getPassableMoveAction");
                        return getMoveAction(pathPlanning, agent.getPosition(), bottomNeighbor2.get(0));
                    }

                    if (number == 2 && !(rightNeighborBuildings.isEmpty())) {

                        //System.out.println(agentInfo.me().getID().getValue() + "getPassableMoveAction");
                        return getMoveAction(pathPlanning, agent.getPosition(), rightNeighbor2.get(0));

                    }

                    if (number == 3 && !(leftNeighborBuildings.isEmpty())) {

                        //System.out.println(agentInfo.me().getID().getValue() + "getPassableMoveAction");
                        return getMoveAction(pathPlanning, agent.getPosition(), leftNeighbor2.get(0));

                    }
                }


            }
        }

        return null;


    }


    private Action calcSupplyAction(Human human, PathPlanning pathPlanning, Collection<EntityID> supplyPositions, EntityID target, boolean isRefill) {
        EntityID position = human.getPosition();
        int size = supplyPositions.size();
        if (supplyPositions.contains(position)) {
            return isRefill ? new ActionRefill() : new ActionRest();
        }
        List<EntityID> firstResult = null;
        while (supplyPositions.size() > 0) {
            pathPlanning.setFrom(position);
            pathPlanning.setDestination(supplyPositions);
            List<EntityID> path = pathPlanning.calc().getResult();
            if (path != null && path.size() > 0) {
                if (firstResult == null) {
                    firstResult = new ArrayList<>(path);
                    if (target == null) {
                        break;
                    }
                }
                EntityID supplyPositionID = path.get(path.size() - 1);
                pathPlanning.setFrom(supplyPositionID);
                pathPlanning.setDestination(target);
                List<EntityID> fromRefugeToTarget = pathPlanning.calc().getResult();
                if (fromRefugeToTarget != null && fromRefugeToTarget.size() > 0) {
                    return new ActionMove(path);
                }
                supplyPositions.remove(supplyPositionID);
                //remove failed
                if (size == supplyPositions.size()) {
                    break;
                }
                size = supplyPositions.size();
            } else {
                break;
            }
        }
        return firstResult != null ? new ActionMove(firstResult) : null;
    }

    private class DistanceSorter implements Comparator<StandardEntity> {
        private StandardEntity reference;
        private WorldInfo worldInfo;

        DistanceSorter(WorldInfo wi, StandardEntity reference) {
            this.reference = reference;
            this.worldInfo = wi;
        }

        public int compare(StandardEntity a, StandardEntity b) {
            int d1 = this.worldInfo.getDistance(this.reference, a);
            int d2 = this.worldInfo.getDistance(this.reference, b);
            return d1 - d2;
        }
    }

}
