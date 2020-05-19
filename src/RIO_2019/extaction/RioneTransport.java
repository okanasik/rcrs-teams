package RIO_2019.extaction;

import adf.agent.action.Action;
import adf.agent.action.ambulance.ActionLoad;
import adf.agent.action.ambulance.ActionRescue;
import adf.agent.action.ambulance.ActionUnload;
import adf.agent.action.common.ActionMove;
import adf.agent.action.common.ActionRest;
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
//import adf.sample.module.complex.GreedySelector.DistanceSorter;

import com.google.common.collect.Lists;
import rescuecore2.config.NoSuchConfigOptionException;
import rescuecore2.misc.Pair;
import rescuecore2.misc.geometry.GeometryTools2D;
import rescuecore2.misc.geometry.Point2D;
import rescuecore2.standard.entities.*;
import rescuecore2.standard.entities.StandardEntityConstants.Fieryness;
import rescuecore2.worldmodel.EntityID;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Random;


import static rescuecore2.standard.entities.StandardEntityURN.POLICE_FORCE;
import static rescuecore2.standard.entities.StandardEntityURN.AMBULANCE_TEAM;
import static rescuecore2.standard.entities.StandardEntityURN.BLOCKADE;
import static rescuecore2.standard.entities.StandardEntityURN.CIVILIAN;
import static rescuecore2.standard.entities.StandardEntityURN.FIRE_BRIGADE;
import static rescuecore2.standard.entities.StandardEntityURN.REFUGE;
import static rescuecore2.standard.entities.StandardEntityURN.BUILDING;;

public class RioneTransport extends ExtAction {
    private PathPlanning pathPlanning;
    
    // calcSearch用
    private Clustering clustering;
    private ArrayList<Integer> movedTime;
    private boolean stopped;
    private ArrayList<EntityID> unsearchedBuildingIDs;
    private int clusterIndex;
    private int changeClusterCycle;
    protected Random random;
    private ArrayList<Point2D> previousLocations;
    private ArrayList<List<EntityID>> previousPaths;

    private int thresholdRest;
    private int kernelTime;

    
  //CommandHistory(前の行動を参照する)
  	HashMap<Integer, Action> CommandHistory;
  	HashMap<Integer, Pair<Integer, Integer>> PositionHistory;
    private EntityID target;

    public RioneTransport(AgentInfo agentInfo, WorldInfo worldInfo, ScenarioInfo scenarioInfo, ModuleManager moduleManager, DevelopData developData) {
        super(agentInfo, worldInfo, scenarioInfo, moduleManager, developData);
        this.target = null;
        this.thresholdRest = developData.getInteger("ActionTransport.rest", 100);
        this.CommandHistory = new HashMap<>();
        this.PositionHistory = new HashMap<>();

        switch  (scenarioInfo.getMode()) {
            case PRECOMPUTATION_PHASE:
                this.pathPlanning = moduleManager.getModule("ActionTransport.PathPlanning", "RIO_2019.module.algorithm.AstarPathPlanning");
                break;
            case PRECOMPUTED:
                this.pathPlanning = moduleManager.getModule("ActionTransport.PathPlanning", "RIO_2019.module.algorithm.AstarPathPlanning");
                break;
            case NON_PRECOMPUTE:
                this.pathPlanning = moduleManager.getModule("ActionTransport.PathPlanning", "RIO_2019.module.algorithm.AstarPathPlanning");
                break;
        }
        
        // calcSearch用
        this.clustering = moduleManager.getModule("ActionTransport.Clustering.Ambulance", "RIO_2019.module.algorithm.RioneKmeansPP");
        unsearchedBuildingIDs = new ArrayList<>();
        movedTime = new ArrayList<>();
        this.changeClusterCycle = 5;
        this.clusterIndex = 0;
        this.random = new Random();
        this.stopped = false;
        this.previousLocations = new ArrayList<>();
        this.previousPaths = new ArrayList<>();
    }

    public ExtAction precompute(PrecomputeData precomputeData) {
        super.precompute(precomputeData);
        if(this.getCountPrecompute() >= 2) {
            return this;
        }
        this.pathPlanning.precompute(precomputeData);
        try {
            this.kernelTime = this.scenarioInfo.getKernelTimesteps();
        }catch (NoSuchConfigOptionException e) {
            this.kernelTime = -1;
        }
        return this;
    }

    public ExtAction resume(PrecomputeData precomputeData) {
        super.resume(precomputeData);
        if(this.getCountResume() >= 2) {
            return this;
        }
        this.pathPlanning.resume(precomputeData);
        try {
            this.kernelTime = this.scenarioInfo.getKernelTimesteps();
        }catch (NoSuchConfigOptionException e) {
            this.kernelTime = -1;
        }
        return this;
    }

    public ExtAction preparate() {
        super.preparate();
        if(this.getCountPreparate() >= 2) {
            return this;
        }
        this.pathPlanning.preparate();
        try {
            this.kernelTime = this.scenarioInfo.getKernelTimesteps();
        }catch (NoSuchConfigOptionException e) {
            this.kernelTime = -1;
        }
        return this;
    }

    public ExtAction updateInfo(MessageManager messageManager){
        super.updateInfo(messageManager);
        if(this.getCountUpdateInfo() >= 2) {
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
        this.target = null; //前回のtarget
        if(target != null) { //今のtarget
            StandardEntity entity = this.worldInfo.getEntity(target);
            if(entity instanceof Human || entity instanceof Area) {
                this.target = target;
                return this;
            }
        }
        return this;
    }
    
    

    @Override
    public ExtAction calc() {
    	//過去の行動を参照する
    	/*if(this.result!=null){
            CommandHistory.put(agentInfo.getTime(),this.result);
		    PositionHistory.put(agentInfo.getTime(),this.worldInfo.getLocation(this.agentInfo.me()));
		  //cantMove
			this.result = cantMove();
			if(this.result != null) {
                return this;
            }
        }*/
    	
    	
        this.result = null;
        AmbulanceTeam agent = (AmbulanceTeam)this.agentInfo.me();
        Human transportHuman = null;
        try {
        	transportHuman = this.agentInfo.someoneOnBoard();
		} catch (Exception e) {
			// TODO: handle exception
			//System.out.println("someoneOnBoard() ダメです");
		}
        if(transportHuman != null) {
            this.result = this.calcUnload(agent, this.pathPlanning, transportHuman, this.target);
            if(this.result != null) {
                return this;
            }
        }               
     // ATのいる場所が燃えてる建物なら隣のエリアに移動する
        StandardEntity position = this.worldInfo.getEntity(agentInfo.getPosition());
        if(position instanceof Building && ((Building)position).isOnFire()&&
        		(((Building)position).getFierynessEnum() == Fieryness.INFERNO||((Building)position).getFierynessEnum() == Fieryness.BURNING)){
        	pathPlanning.setFrom(position.getID());
        	pathPlanning.setDestination(((Building) position).getNeighbours());        	
        	List<EntityID> path = pathPlanning.calc().getResult();
        	if (path != null && path.size() > 0) {
                this.result = calcActionMove(path);  //Buildingからでる（隣のroad（getNeighbor））)
        	}
        	return this;
        }               
        //this.resultが指定されない時
        if(this.needRest(agent)) {
            EntityID areaID = this.convertArea(this.target);  //(convert変換する)targetのposition（areaID）を返す
            ArrayList<EntityID> targets = new ArrayList<>();
            if(areaID != null) {
                targets.add(areaID);
            }
            this.result = this.calcRefugeAction(agent, this.pathPlanning, targets, false);
            if(this.result != null) {
                return this;
            }
        }
        if(this.target != null) {
            this.result = this.calcRescue(agent, this.pathPlanning, this.target);
        }
        /*
        if(this.result == null) {
        	Collection<StandardEntity> refuges = this.worldInfo.getEntitiesOfType(StandardEntityURN.REFUGE);
    		if (refuges.isEmpty()) return null; //Refugeがなければnull
        }
        */
        if(this.result == null){ 
        	this.result = calcSearch();
        	return this;
        }
        return this;
    }
    
    private Action calcSearch(){
    	if (agentInfo.getTime() < scenarioInfo.getKernelAgentsIgnoreuntil()) {
            return null;
        }
        return getSearchAction(pathPlanning,this.agentInfo.getPosition(),this.unsearchedBuildingIDs);
    }

    private Action getSearchAction(PathPlanning pathPlanning, EntityID from,  Collection<EntityID> targets){
    	pathPlanning.setFrom(from);
    	pathPlanning.setDestination(targets);
    	List<EntityID> path = pathPlanning.calc().getResult();
    	previousPaths.add(path);

    	if(previousPaths.size()<2 || !isStopped(previousPaths.get(0),previousPaths.get(1))){
    		if (path != null && path.size() > 0){
    			StandardEntity entity = this.worldInfo.getEntity(path.get(path.size() - 1));
    			if (entity instanceof Building)
    			{
    				if (entity.getStandardURN() != StandardEntityURN.REFUGE)
    				{
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

    // 止まってる判定はtrue、止まってなければfalse
    private boolean isStopped(List<EntityID> path1, List<EntityID> path2) {
    	Human agent = (Human)this.agentInfo.me();
    	previousLocations.add(new Point2D(agent.getX(),agent.getY()));//移動するときの場所を記録(0が現在地)
    	
    	if (path1 == null || path2 == null){
    		return false;
    	}
    	if (path1.size() != path2.size()) {
    		return false;
    	}else{
    		for (int i = 0; i < path1.size(); i++) {
    			EntityID id1 = path1.get(i);
    			EntityID id2 = path2.get(i);
    			if (!id1.equals(id2))
    				return false;
    		}
    	}

    	if(previousLocations.size()>2) {
    		return withinRange(previousLocations.get(0),previousLocations.get(1),previousLocations.get(2));
    	}
    	return false;
    }
    
    private boolean  withinRange(Point2D position1,Point2D position2,Point2D position3) {
        int range = 30000;
  
        double dist1 = GeometryTools2D.getDistance(position1, position2);
        double dist2 = GeometryTools2D.getDistance(position1, position3);

        if (dist1 < range && dist2 < range) {
            return true;
        }

        return false;
    }
    private void reset(){
    	this.unsearchedBuildingIDs.clear();
    	this.previousPaths.clear();
        this.previousLocations.clear();

    	if((this.agentInfo.getTime()!=0 && (this.agentInfo.getTime()%this.changeClusterCycle)==0)||stopped){
    		this.stopped=false;
    		this.clusterIndex = random.nextInt(clustering.getClusterNumber());
    		this.changeClusterCycle = random.nextInt(16) + 15;//変更

    	}
    	Collection<StandardEntity> clusterEntities = new ArrayList<>();
    	if(clustering!=null) {
    		clusterEntities.addAll(this.clustering.getClusterEntities(clusterIndex));
    	}

    	if(clusterEntities != null && clusterEntities.size() > 0) {
    		for(StandardEntity entity : clusterEntities) {
    			if(entity instanceof Building && entity.getStandardURN() != REFUGE) {
    				this.unsearchedBuildingIDs.add(entity.getID());
    			}
    		}
    	}else{
    		this.unsearchedBuildingIDs.addAll(this.worldInfo.getEntityIDsOfType(BUILDING));
    	}
    }
    
    private Action calcActionMove(List<EntityID> path) {
    	previousPaths.add(path);
    	if(previousPaths.size()<2 || !isStopped(previousPaths.get(0),previousPaths.get(1))){
    		if (path != null && path.size() > 0){
    			StandardEntity entity = this.worldInfo.getEntity(path.get(path.size() - 1));
    			if (entity instanceof Building){
    				if (entity.getStandardURN() != StandardEntityURN.REFUGE && ((Building)entity).isOnFire()){
    					path.remove(path.size() - 1);
    				}
    			}
    			movedTime.add(agentInfo.getTime());//動いた時のTimeを記録
    			if(path.size()>0) {
    				return new ActionMove(path);
    			}
    		}
    		return null;
    	}
    	return null;
    }
       
    
    
    private Action calcRescue(AmbulanceTeam agent, PathPlanning pathPlanning, EntityID targetID) {
        StandardEntity targetEntity = this.worldInfo.getEntity(targetID);
        if(targetEntity == null) {
            return null;
        }
        EntityID agentPosition = agent.getPosition();
        if(targetEntity instanceof Human) {
            Human human = (Human) targetEntity;
            if (!human.isPositionDefined()) {
                return null;
            }
            if (human.isHPDefined() && human.getHP() == 0) {
                return null;
            }
            EntityID targetPosition = human.getPosition();
            if (agentPosition.getValue() == targetPosition.getValue()) {
                if (human.isBuriednessDefined() && human.getBuriedness() > 0) {
                    return new ActionRescue(human);
                } else if (human.getStandardURN() == CIVILIAN) {
                    return new ActionLoad(human.getID());
                }
            } else {
                List<EntityID> path = pathPlanning.getResult(agentPosition, targetPosition);
                if (path != null && path.size() > 0) {
                    return calcActionMove(path);
                }
            }
            return null;
        }
        /* え？いらんくね？
        if(targetEntity instanceof Blockade) {
            Blockade blockade = (Blockade) targetEntity;
            if(blockade.isPositionDefined()) {
                targetEntity = this.worldInfo.getEntity(blockade.getPosition());
            }
        }
        */
        if(targetEntity instanceof Area) {
            List<EntityID> path = pathPlanning.getResult(agentPosition, targetEntity.getID());
            if (path != null && path.size() > 0) {
                this.result = calcActionMove(path);
            }
        }
        return null;
    }
    
    private Action calcUnload(AmbulanceTeam agent, PathPlanning pathPlanning, Human transportHuman, EntityID targetID) {
        if (transportHuman == null) {
            return null;
        }
        if (transportHuman.isHPDefined() && transportHuman.getHP() == 0) {
            return new ActionUnload();
        }
        EntityID agentPosition = agent.getPosition();
        if(targetID == null || transportHuman.getID().getValue() == targetID.getValue()) {
            StandardEntity position = this.worldInfo.getEntity(agentPosition);
            if (position != null && position.getStandardURN() == REFUGE) {
                return new ActionUnload();
            } else {
                pathPlanning.setFrom(agentPosition);
                pathPlanning.setDestination(this.worldInfo.getEntityIDsOfType(REFUGE));
                List<EntityID> path = pathPlanning.calc().getResult();
                if (path != null && path.size() > 0) {
                    return calcActionMove(path);
                }
            }
        }
        if(targetID == null) {
            return null;
        }
        StandardEntity targetEntity = this.worldInfo.getEntity(targetID);
        if(targetEntity != null && targetEntity.getStandardURN() == BLOCKADE) {
            Blockade blockade = (Blockade)targetEntity;
            if(blockade.isPositionDefined()) {
                targetEntity = this.worldInfo.getEntity(blockade.getPosition());
            }
        }
        if(targetEntity instanceof Area) {
            if (agentPosition.getValue() == targetID.getValue()) {
                return new ActionUnload();
            } else {
                pathPlanning.setFrom(agentPosition);
                pathPlanning.setDestination(targetID);
                List<EntityID> path = pathPlanning.calc().getResult();
                if (path != null && path.size() > 0) {
                    return calcActionMove(path);
                }
            }
        } else if(targetEntity instanceof Human) {
            Human human = (Human)targetEntity;
            if(human.isPositionDefined()) {
                return calcRefugeAction(agent, pathPlanning, Lists.newArrayList(human.getPosition()), true);
            }
            pathPlanning.setFrom(agentPosition);
            pathPlanning.setDestination(this.worldInfo.getEntityIDsOfType(REFUGE));
            List<EntityID> path = pathPlanning.calc().getResult();
            if (path != null && path.size() > 0) {
                return calcActionMove(path);
            }
        }
        return null;
    }

    private boolean needRest(Human agent) {
        int hp = agent.getHP();
        int damage = agent.getDamage();
        if(hp == 0 || damage == 0) {
            return false;
        }
        int activeTime = (hp / damage) + ((hp % damage) != 0 ? 1 : 0);
        if(this.kernelTime == -1) {
            try {
                this.kernelTime = this.scenarioInfo.getKernelTimesteps();
            }catch (NoSuchConfigOptionException e) {
                this.kernelTime = -1;
            }
        }
        return damage >= this.thresholdRest || (activeTime + this.agentInfo.getTime()) < this.kernelTime;
    }

    private EntityID convertArea(EntityID targetID) {
        StandardEntity entity = this.worldInfo.getEntity(targetID);
        if(entity == null) {
            return null;
        }
        if(entity instanceof Human) {
            Human human = (Human) entity;
            if(human.isPositionDefined()) {
                EntityID position = human.getPosition();
                if(this.worldInfo.getEntity(position) instanceof Area) {
                    return position;
                }
            }
        }else if(entity instanceof Area) {
            return targetID;
        }else if(entity.getStandardURN() == BLOCKADE) {
            Blockade blockade = (Blockade)entity;
            if(blockade.isPositionDefined()) {
                return blockade.getPosition();
            }
        }
        return null;
    }

    private Action calcRefugeAction(Human human, PathPlanning pathPlanning, Collection<EntityID> targets, boolean isUnload) {
        EntityID position = human.getPosition();
        Collection<EntityID> refuges = this.worldInfo.getEntityIDsOfType(StandardEntityURN.REFUGE);
        int size = refuges.size();
        if(refuges.contains(position)) {
            return isUnload ? new ActionUnload() : new ActionRest();
        }
        List<EntityID> firstResult = null;
        while(refuges.size() > 0) {
            pathPlanning.setFrom(position);
            pathPlanning.setDestination(refuges);
            List<EntityID> path = pathPlanning.calc().getResult();
            if (path != null && path.size() > 0) {
                if (firstResult == null) {
                    firstResult = new ArrayList<>(path);
                    if(targets == null || targets.isEmpty()) {
                        break;
                    }
                }
                EntityID refugeID = path.get(path.size() - 1);
                pathPlanning.setFrom(refugeID);
                pathPlanning.setDestination(targets);
                List<EntityID> fromRefugeToTarget = pathPlanning.calc().getResult();
                if (fromRefugeToTarget != null && fromRefugeToTarget.size() > 0) {
                    return calcActionMove(path);
                }
                refuges.remove(refugeID);
                //remove failed
                if (size == refuges.size()) {
                    break;
                }
                size = refuges.size();
            } else {
                break;
            }
        }
        return firstResult != null ? calcActionMove(firstResult) : null;
    }
    
    
    

        
   /* 
    private Action cantMove() {
   	 PathPlanning pathPlanning = this.moduleManager.getModule("ActionTransport.PathPlanning");
        StandardEntity position = this.worldInfo.getEntity(agentInfo.getPosition());
        pathPlanning.setFrom(position.getID());
        
   	if(CommandHistory.get(agentInfo.getTime()-1) != null && CommandHistory.get(agentInfo.getTime()-2) != null){
   		if(CommandHistory.get(agentInfo.getTime()-1) instanceof ActionMove &&
       		CommandHistory.get(agentInfo.getTime()-2) instanceof ActionMove){
   			if(PositionHistory.get(agentInfo.getTime()-1) == PositionHistory.get(agentInfo.getTime()-2)){
   				System.out.println("0haru");//位置移動していなかったら
   				if (position instanceof Area) {	
   					pathPlanning.setDestination(((Area) position).getNeighbours());
   				    List<EntityID> path = pathPlanning.calc().getResult();
   				    System.out.println("1haru");
   				    if (path != null && path.size() > 0) {
  					        return new ActionMove(path);
  					    }
   				}
   			}
   		}else if(CommandHistory.get(agentInfo.getTime()-1) instanceof ActionRescue
   					&& position instanceof Building && ((Building)position).isOnFire()){
   			pathPlanning.setDestination(((Building) position).getNeighbours());
   			List<EntityID> path = pathPlanning.calc().getResult();
   			System.out.println("2haru");
   			if (path != null && path.size() > 0) {
   				return new ActionMove(path); //Buildingからでる（隣のroad（getNeighbor））)
   			}
   		}
   	}
   	return null;
	}*/ 
    
}
