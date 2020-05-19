package RIO_2019.module.complex.self;

import adf.agent.action.Action;
import adf.agent.action.common.ActionMove;
import adf.agent.communication.MessageManager;
import adf.agent.communication.standard.bundle.MessageUtil;
import adf.agent.communication.standard.bundle.information.*;
import adf.agent.develop.DevelopData;
import adf.agent.info.AgentInfo;
import adf.agent.info.ScenarioInfo;
import adf.agent.info.WorldInfo;
import adf.agent.module.ModuleManager;
import adf.agent.precompute.PrecomputeData;
import adf.component.communication.CommunicationMessage;
import adf.component.module.algorithm.Clustering;
import adf.component.module.algorithm.PathPlanning;
import adf.component.module.complex.Search;
import rescuecore2.misc.geometry.Point2D;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;

import java.util.*;

import static rescuecore2.standard.entities.StandardEntityURN.*;

import adf.launcher.ConsoleOutput;

import rescuecore2.misc.geometry.Vector2D;

import rescuecore2.misc.geometry.Line2D;
import rescuecore2.worldmodel.ChangeSet;
import java.awt.Polygon;
import java.awt.geom.GeneralPath;
import java.awt.geom.PathIterator;
import rescuecore2.misc.geometry.GeometryTools2D;

public class RIOSearchForFire extends Search {
	private PathPlanning pathPlanning;
	private Clustering clustering;

	private EntityID result;


	private boolean stopped;
	private ArrayList<EntityID> unsearchedBuildingIDs;
	private int clusterIndex;
	private int changeClusterCycle;
	protected Random random;
	private ArrayList<Point2D> previousLocations;
	private ArrayList<Point2D> previousLocations2;
	private ArrayList<List<EntityID>> previousPaths;
	private ArrayList<List<EntityID>> paths;


	

	public RIOSearchForFire(AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager,
			DevelopData developData) {
		super(ai, wi, si, moduleManager, developData);

		unsearchedBuildingIDs = new ArrayList<>();
		this.changeClusterCycle = 5;
		this.clusterIndex = 0;
		this.random = new Random();
		this.stopped = false;
		this.previousLocations = new ArrayList<>();
		this.previousLocations2 = new ArrayList<>();
		this.previousPaths = new ArrayList<>();
		this.paths = new ArrayList<>();

		//lastSearchPath = null;
		//lastPosition = null;



		StandardEntityURN agentURN = ai.me().getStandardURN();
		if (agentURN == AMBULANCE_TEAM) {
			this.pathPlanning = moduleManager.getModule("SearchForFire.PathPlanning.Ambulance", "RIO_2019.module.algorithm.AstarPathPlanning");
			this.clustering = moduleManager.getModule("SearchForFire.Clustering.Ambulance", "RIO_2019.module.algorithm.RioneKmeansPP");
		} else if (agentURN == FIRE_BRIGADE) {
			this.pathPlanning = moduleManager.getModule("SearchForFire.PathPlanning.Fire", "RIO_2019.module.algorithm.AstarPathPlanning");
			this.clustering = moduleManager.getModule("SearchForFire.Clustering.Fire", "RIO_2019.module.algorithm.RioneKmeansPP");
		} else if (agentURN == POLICE_FORCE) {
			this.pathPlanning = moduleManager.getModule("SearchForFire.PathPlanning.Police", "RIO_2019.module.algorithm.AstarPathPlanning");
			this.clustering = moduleManager.getModule("SearchForFire.Clustering.Police", "RIO_2019.module.algorithm.RioneKmeansPP");
		}
		registerModule(this.clustering);
		registerModule(this.pathPlanning);


		this.clusterIndex = 0;
		this.random = new Random();
		this.stopped = false;
	}

	@Override
	public Search updateInfo(MessageManager messageManager) {
		super.updateInfo(messageManager);
		if (this.getCountUpdateInfo() >= 2) {
			return this;
		}

		if (this.unsearchedBuildingIDs.isEmpty()) {
			useCluster = false;
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


	private void reset(){
        this.unsearchedBuildingIDs.clear();
        this.previousPaths.clear();
        this.previousLocations.clear();
        
        if((this.agentInfo.getTime()!=0 && (this.agentInfo.getTime()%this.changeClusterCycle)==0)||stopped){
        	this.stopped=false;
        	this.clusterIndex = random.nextInt(clustering.getClusterNumber());
        	//this.changeClusterCycle = random.nextInt(71) + 30;
        	
        }

        Collection<StandardEntity> clusterEntities = new ArrayList<>();
            
            clusterEntities.addAll(this.clustering.getClusterEntities(clusterIndex));

        
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
	
	@Override
	public Search calc() {// パスの生成、つっかえたら作りなおす
		if (agentInfo.getTime() < scenarioInfo.getKernelAgentsIgnoreuntil()) {
			result = null;
			return this;
		}
		this.result = getMovePosition(pathPlanning,this.agentInfo.getPosition(),this.unsearchedBuildingIDs);
		return this;
	}
	//nishida
	private EntityID getMovePosition(PathPlanning pathPlanning, EntityID from, Collection<EntityID> targets){
		pathPlanning.setFrom(from);
		pathPlanning.setDestination(targets);
		List<EntityID> path = pathPlanning.calc().getResult();
		previousPaths.add(path);

		if(previousPaths.size()<2 || !isStopped(previousPaths.get(0),previousPaths.get(1))){//止まってるかどうか判定
			if (path != null && path.size() > 0){
				StandardEntity entity = this.worldInfo.getEntity(path.get(path.size() - 1));
				if (entity instanceof Building)
				{
					if (entity.getStandardURN() != StandardEntityURN.REFUGE)
					{
						path.remove(path.size() - 1);
					}
				}
				if(path.size()>0) {
				//movedTime.add(agentInfo.getTime());//動いた時のTimeを記録
				return path.get(path.size() - 1);
				}
			}
			return null;
		}
		this.stopped = true;
		reset();
		return null;
	}
	//nishida
	/*private boolean isStopped(EntityID from) {
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
	}*/

	private boolean isStopped(List<EntityID> path1, List<EntityID> path2) {
		Human agent = (Human)this.agentInfo.me();
		previousLocations.add(new Point2D(agent.getX(),agent.getY()));//移動するときの場所を記録(0が現在地)
		if (path1 == null || path2 == null) {
			return false;
			//System.out.println("flag,false1");
		}

		if (path1.size() != path2.size()) {
			//System.out.println("flag,false2");
			return false;
		}else {
			for (int i = 0; i < path1.size(); i++) {
				EntityID id1 = path1.get(i);
				EntityID id2 = path2.get(i);
				if (!id1.equals(id2)) {
					return false;
					//System.out.println("flag,false3");
				}
			}
		}

		if(previousLocations.size()>2) {
			//System.out.println("flag,true");
			return withinRange(previousLocations.get(0),previousLocations.get(1),previousLocations.get(2));
		}
		//System.out.println("isStopped,false");
		return false;

	}
	private boolean  withinRange(Point2D position1,Point2D position2,Point2D position3) {
		int range = 30000;

		double dist1 = GeometryTools2D.getDistance(position1, position2);
		double dist2 = GeometryTools2D.getDistance(position1, position3);
		if (dist1 < range && dist2 < range) {
			//System.out.println("isStopped,true");
			return true;
		}
		//System.out.println("isStopped,false");
		return false;
	}

	protected boolean useCluster = true;

	@Override
	public EntityID getTarget() {
		return this.result;
	}

	@Override
	public Search precompute(PrecomputeData precomputeData) {
		super.precompute(precomputeData);
		if (this.getCountPrecompute() >= 2) {
			return this;
		}
		// this.pathPlanning.precompute(precomputeData);
		// this.clustering.precompute(precomputeData);
		return this;
	}

	@Override
	public Search resume(PrecomputeData precomputeData) {
		super.resume(precomputeData);
		if (this.getCountResume() >= 2) {
			return this;
		}
		// this.worldInfo.requestRollback();
		// this.pathPlanning.resume(precomputeData);
		// this.clustering.resume(precomputeData);
		preparate();
		return this;
	}

	@Override
	public Search preparate() {
		super.preparate();
		if (this.getCountPreparate() >= 2) {
			return this;
		}
		this.worldInfo.requestRollback();
		this.pathPlanning.preparate();
		this.clustering.preparate();
		reset();
		return this;
	}

	protected StandardEntityURN getAgentType() {
		StandardEntity se = worldInfo.getEntity(agentInfo.getID());
		if (se instanceof PoliceForce) {
			return StandardEntityURN.POLICE_FORCE;
		} else if (se instanceof FireBrigade) {
			return StandardEntityURN.FIRE_BRIGADE;
		} else if (se instanceof AmbulanceTeam) {
			return StandardEntityURN.AMBULANCE_TEAM;
		} else if (se instanceof PoliceOffice) {
			return StandardEntityURN.POLICE_OFFICE;
		} else if (se instanceof FireStation) {
			return StandardEntityURN.FIRE_STATION;
		} else if (se instanceof AmbulanceCentre) {
			return StandardEntityURN.AMBULANCE_CENTRE;
		} else {
			return null;
		}
	}
	private boolean isPathEqual(List<EntityID> path1, List<EntityID> path2) {// 全く同じパスかどうか判定（同じでtrue）
		if (path1 == null || path2 == null)
			return false;

		if (path1.size() != path2.size())
			return false;

		for (int i = 0; i < path1.size(); i++) {
			EntityID id1 = path1.get(i);
			EntityID id2 = path2.get(i);
			if (!id1.equals(id2))
				return false;
		}

		return true;
	}
}