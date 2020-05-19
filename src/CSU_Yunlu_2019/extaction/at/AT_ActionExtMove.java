package CSU_Yunlu_2019.extaction.at;

import CSU_Yunlu_2019.util.CSU_Ambulancehelper.CSU_SelectorTargetByDis;
import adf.agent.action.Action;
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
import adf.component.module.algorithm.PathPlanning;
import rescuecore2.config.NoSuchConfigOptionException;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;

import java.awt.*;
import java.util.*;
import java.util.List;

public class AT_ActionExtMove extends ExtAction {

	private PathPlanning pathPlanning;

	private int thresholdRest;
	private int kernelTime;

	private EntityID target;
	/**
	 * new add to solve the stuked
	 */
	private rescuecore2.misc.Pair<Integer, Integer> selfLocation;
	private Point lastlastPosition=null;
	private Point lastPosition=null;
	private Point nowPosition=null;

	private EntityID StuckedTarget;
	private EntityID lastTarget;
	private final int STUCK_THRESHOLD = 2000;//threshold of stuck

	private Random random = new Random();

	public AT_ActionExtMove(AgentInfo agentInfo, WorldInfo worldInfo, ScenarioInfo scenarioInfo,
                               ModuleManager moduleManager, DevelopData developData) {
		super(agentInfo, worldInfo, scenarioInfo, moduleManager, developData);
		this.target = null;
		this.thresholdRest = developData.getInteger("ActionExtMove.rest", 100);

		switch (scenarioInfo.getMode()) {
		case PRECOMPUTATION_PHASE:
			this.pathPlanning = moduleManager.getModule("ActionExtMove.PathPlanning",
					"adf.sample.module.algorithm.SamplePathPlanning");
			break;
		case PRECOMPUTED:
			this.pathPlanning = moduleManager.getModule("ActionExtMove.PathPlanning",
					"adf.sample.module.algorithm.SamplePathPlanning");
			break;
		case NON_PRECOMPUTE:
			this.pathPlanning = moduleManager.getModule("ActionExtMove.PathPlanning",
					"adf.sample.module.algorithm.SamplePathPlanning");
			break;
		}
		selfLocation = worldInfo.getLocation(agentInfo.getID());
		this.nowPosition=new Point(selfLocation.first(),selfLocation.second());
		this.StuckedTarget=null;
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
		return this;
	}

	@Override
	public ExtAction setTarget(EntityID target) {
		this.target = null;
		StandardEntity entity = this.worldInfo.getEntity(target);

		if (entity != null) {
			if (entity.getStandardURN().equals(StandardEntityURN.BLOCKADE)) {
				entity = this.worldInfo.getEntity(((Blockade) entity).getPosition());
			} else if (entity instanceof Human) {
				entity = this.worldInfo.getPosition((Human) entity);
			}
			if (entity != null && entity instanceof Area) {
				this.target = entity.getID();
			}
		}
		return this;
	}

	@Override
	public ExtAction calc() {
		lastlastPosition=lastPosition;
		lastPosition=nowPosition;
		nowPosition=new Point(worldInfo.getLocation(agentInfo.getID()).first(),worldInfo.getLocation(agentInfo.getID()).second());
		/*
		if(lastlastPosition!=null && lastPosition!=null && nowPosition!=null) {
			System.out.println(this.agentInfo.getID() + " " + lastlastPosition.getLocation().x + " " + lastlastPosition.getLocation().y);
			System.out.println(this.agentInfo.getID() + " " + lastPosition.getLocation().x + " " + lastPosition.getLocation().y);
			System.out.println(this.agentInfo.getID() + " " + nowPosition.getLocation().x + " " + nowPosition.getLocation().y);
		}
		*/
		if(CalStucked()){
			if(lastTarget!=null) {
				//System.out.println(this.agentInfo.getID()+"Stucked***********");
				StuckedTarget = lastTarget;
			}else{
				StuckedTarget = null;
			}
		}

		this.result = null;
		Human agent = (Human) this.agentInfo.me();
		/********************************/
		Action randomaction =randomWalk();

		if (this.needRest(agent)) {
			this.result = this.calcRest(agent, this.pathPlanning, this.target);
			lastTarget=null;
			if (this.result != null) {
				return this;
			}
		}
		if (this.target == null) {//如果没有目标 就乱走
			if(randomaction!=null) {
				lastTarget=null;
				//System.out.println(this.agentInfo.getID()+"randomwolk*********");
				this.result = randomaction;
			}
			return this;
		}
		if(StuckedTarget!=null) {
			if (CalEqual(this.target, StuckedTarget)) {
//				System.out.print("----equal----");
				lastTarget=StuckedTarget;
				//System.out.println(this.agentInfo.getID()+"randomwolk*********");
				this.result = randomaction;
				return this;
			} else {
				StuckedTarget=null;
				this.pathPlanning.setFrom(agent.getPosition());
				this.pathPlanning.setDestination(this.target);
				lastTarget=target;
				List<EntityID> path = this.pathPlanning.calc().getResult();
				if (path != null && path.size() > 0) {
					this.result = new ActionMove(path);
				}
				return this;
			}
		}else{
			this.pathPlanning.setFrom(agent.getPosition());
			this.pathPlanning.setDestination(this.target);
			lastTarget=target;
			List<EntityID> path = this.pathPlanning.calc().getResult();
			if (path != null && path.size() > 0) {
				this.result = new ActionMove(path);
			}
			return this;
		}
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

	private Action calcRest(Human human, PathPlanning pathPlanning, EntityID target) {
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
					if (target == null) {
						break;
					}
				}
				EntityID refugeID = path.get(path.size() - 1);
				pathPlanning.setFrom(refugeID);
				pathPlanning.setDestination(target);
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

	public boolean CalStucked(){
		if(this.lastlastPosition!=null&&this.lastPosition!=null){
			int moveDistance1 = CSU_SelectorTargetByDis.getDistance.distance(lastlastPosition,lastPosition);
			int moveDistance2 = CSU_SelectorTargetByDis.getDistance.distance(lastlastPosition,nowPosition);
			int moveDistance3 = CSU_SelectorTargetByDis.getDistance.distance(lastPosition,nowPosition);
			if(moveDistance1<STUCK_THRESHOLD||moveDistance2<STUCK_THRESHOLD||moveDistance3<STUCK_THRESHOLD){
				return true;
			}
		}
		return false;
	}

	/**
	 * 返回是否移动过近
	 * @param from
	 * @param end
	 * @return
	 */
	public boolean CalNear(EntityID from,EntityID end){
		rescuecore2.misc.Pair<Integer, Integer> fromLocation = worldInfo.getLocation(from);
		rescuecore2.misc.Pair<Integer, Integer> endLocation = worldInfo.getLocation(end);
		Point frompoint = new Point(fromLocation.first(), fromLocation.second());//update the position
		Point endpoint = new Point(endLocation.first(), endLocation.second());
		int moveDistance = CSU_SelectorTargetByDis.getDistance.distance(frompoint,endpoint);
		if (moveDistance <= STUCK_THRESHOLD) {
			return true;
		}else{
			return false;
		}
	}
	public boolean CalEqual(EntityID from,EntityID end){
		return from.getValue() == end.getValue();
	}

	public Action randomWalk() { ///

		final int cnt_limit = 1000;

		Collection<StandardEntity> inRnage = worldInfo.getObjectsInRange(agentInfo.getID(), 50000);
		EntityID position = agentInfo.getPosition();

		Object[] array = inRnage.toArray();
		//System.out.println(array.length);
		/*System.out.println("Random walk: inRnage==null? " + (inRnage==null) + 
							"; array.size = " + array.length + " random entity");*/
		if (array.length>0) {
			StandardEntity entity = (StandardEntity) array[random.nextInt(array.length)];
			List<EntityID> path = null;
			/*
			 * 挨个寻找可到达的 未起火的
			 */
			for (int cnt=0;cnt<cnt_limit;cnt++){
				if (!(entity instanceof Area)) {
					entity = (StandardEntity) array[random.nextInt(array.length)];
					continue;
				}
				if (entity instanceof Building && ((Building) entity).isOnFire()) {
					entity = (StandardEntity) array[random.nextInt(array.length)];
					continue;
				}
				if (entity.getID().getValue() == agentInfo.getID().getValue()) {
					entity = (StandardEntity) array[random.nextInt(array.length)];
					continue;
				}
				if (position.getValue() == entity.getID().getValue()) {
					entity = (StandardEntity) array[random.nextInt(array.length)];
					continue;
				}
				Area nearArea = (Area) entity;
				if (!nearArea.isBlockadesDefined()) {
					entity = (StandardEntity) array[random.nextInt(array.length)];
					continue;
				}
				//System.out.println("nearArea BlockadesDefined");

				pathPlanning.setFrom(position);
				pathPlanning.setDestination(entity.getID());
				path = this.pathPlanning.calc().getResult();
				break;
			}
			if (path == null) {
				//System.out.println("Random walk failed.");
				return null;
			}
			//System.out.println("Random walk succeed.");
			return new ActionMove(path);
		}
		return null;
	}
}

