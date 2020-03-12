package RIO_2019.module.complex.center;

import adf.agent.communication.MessageManager;
import adf.agent.communication.standard.bundle.MessageUtil;
import adf.agent.communication.standard.bundle.centralized.CommandFire;
import adf.agent.communication.standard.bundle.centralized.MessageReport;
import adf.agent.communication.standard.bundle.information.MessageBuilding;
import adf.agent.communication.standard.bundle.information.MessageFireBrigade;
import adf.agent.develop.DevelopData;
import adf.agent.info.AgentInfo;
import adf.agent.info.ScenarioInfo;
import adf.agent.info.WorldInfo;
import adf.agent.module.ModuleManager;
import adf.agent.precompute.PrecomputeData;
import adf.component.centralized.CommandPicker;
import adf.component.communication.CommunicationMessage;
import adf.component.module.complex.FireTargetAllocator;
import RIO_2019.module.algorithm.RioFireClustering;
import rescuecore2.config.Config;
import rescuecore2.standard.entities.*;
import rescuecore2.standard.kernel.comms.ChannelCommunicationModel;
import rescuecore2.worldmodel.EntityID;

import static rescuecore2.standard.entities.StandardEntityURN.AMBULANCE_TEAM;
import static rescuecore2.standard.entities.StandardEntityURN.FIRE_BRIGADE;
import static rescuecore2.standard.entities.StandardEntityURN.FIRE_STATION;
import static rescuecore2.standard.entities.StandardEntityURN.POLICE_FORCE;

import java.util.*;

public class RIOFireTargetAllocator extends FireTargetAllocator {

	private Collection<EntityID> priorityBuildings;
	private Collection<EntityID> targetBuildings;
	private RioFireClustering fireClustering;
	private CommandPicker picker;

	private Map<EntityID, FireBrigadeInfo> agentInfoMap;

	private int maxWater;
	private int maxPower;

	//Radio
	private int channelMax = 0;
	private int bandwidth;
	private int devidedBandwidth;
	private int moveDistance;


	public RIOFireTargetAllocator(AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager,
			DevelopData developData) {
		super(ai, wi, si, moduleManager, developData);
		this.priorityBuildings = new HashSet<>();
		this.targetBuildings = new HashSet<>();
		this.agentInfoMap = new HashMap<>();
		this.maxWater = si.getFireTankMaximum();
		this.maxPower = si.getFireExtinguishMaxSum();

		switch (scenarioInfo.getMode())
		{
		case PRECOMPUTATION_PHASE:
		case PRECOMPUTED:
			this.picker = moduleManager.getCommandPicker(
					"TacticsFireStation.CommandPicker",
					"adf.sample.centralized.CommandPickerFire");
			break;
		case NON_PRECOMPUTE:
			this.picker = moduleManager.getCommandPicker(
					"TacticsFireStation.CommandPicker",
					"adf.sample.centralized.CommandPickerFire");

		}


		this.fireClustering=moduleManager.getModule("FireTargetAllocator.SampleFireClustering", "RIO_2019.module.algorithm.RioFireClustering");
		registerModule(this.fireClustering);

		//Radio
		Config config = this.scenarioInfo.getRawConfig();
		this.bandwidth = config.getIntValue(ChannelCommunicationModel.PREFIX+1+".bandwidth");
		this.channelMax = this.scenarioInfo.getCommsChannelsCount();
		int numAgents = this.worldInfo.getEntitiesOfType(AMBULANCE_TEAM,FIRE_BRIGADE,POLICE_FORCE).size();
		int numCenter = this.worldInfo.getEntitiesOfType(FIRE_STATION).size();
		this.devidedBandwidth = bandwidth / (numAgents + numCenter);
	}

	@Override
	public FireTargetAllocator resume(PrecomputeData precomputeData) {
		super.resume(precomputeData);
		if (this.getCountResume() >= 2) {
			return this;
		}
		for (EntityID id : this.worldInfo.getEntityIDsOfType(StandardEntityURN.FIRE_BRIGADE)) {
			this.agentInfoMap.put(id, new FireBrigadeInfo(id));
		}
		return this;
	}

	@Override
	public FireTargetAllocator preparate() {
		super.preparate();
		if (this.getCountPrecompute() >= 2) {
			return this;
		}
		for (EntityID id : this.worldInfo.getEntityIDsOfType(StandardEntityURN.FIRE_BRIGADE)) {
			this.agentInfoMap.put(id, new FireBrigadeInfo(id));
		}
		return this;
	}

	/*@Override
	public Map<EntityID, EntityID> getResult(){
		int limitBandwidth = devidedBandwidth;
		Map<EntityID, EntityID> results = new HashMap();
		Map<EntityID, EntityID> map = this.convert(this.agentInfoMap);
		for(EntityID m : map){
			CommunicationMessage message = this.picker.setAllocatorResult(this.convert(m)).calc().getResult();
			if(limitBandwidth - message.toByteArray().length + 1 > 0){
				limitBandwidth -= message.toByteArray().length + 1;
				results.add()
			}
        }
		return this.convert(this.agentInfoMap);

		final class in{
			private in(){

			}

		}
	}*/
	@Override
	public Map<EntityID, EntityID> getResult(){
		return this.convert(this.agentInfoMap);
	}

	@Override
	public FireTargetAllocator calc(){
		int currentTime = this.agentInfo.getTime();
		List<StandardEntity> useAgents = this.getActionAgents(this.agentInfoMap);
		List<StandardEntity> agents = calcInFireCluster(currentTime,useAgents);
		Collection<EntityID> removes = new ArrayList<>();



		for (EntityID target : this.priorityBuildings) {
			if (agents.size() > 0) {
				StandardEntity targetEntity = this.worldInfo.getEntity(target);
				if (targetEntity != null) {
					agents.sort(new DistanceSorter(this.worldInfo, targetEntity));
					StandardEntity result = agents.get(0);
					agents.remove(0);
					FireBrigadeInfo info = this.agentInfoMap.get(result.getID());
					if (info != null) {
						info.canNewAction = false;
						info.target = target;
						info.commandTime = currentTime;
						this.agentInfoMap.put(result.getID(), info);
						removes.add(target);
					}
				}
			}
		}
		this.priorityBuildings.removeAll(removes);
		removes.clear();
		for (EntityID target : this.targetBuildings) {
			if (agents.size() > 0) {
				StandardEntity targetEntity = this.worldInfo.getEntity(target);
				if (targetEntity != null) {
					agents.sort(new DistanceSorter(this.worldInfo, targetEntity));
					StandardEntity result = agents.get(0);
					agents.remove(0);
					FireBrigadeInfo info = this.agentInfoMap.get(result.getID());
					if (info != null) {
						info.canNewAction = false;
						info.target = target;
						info.commandTime = currentTime;
						this.agentInfoMap.put(result.getID(), info);
						removes.add(target);
					}
				}
			}
		}
		this.targetBuildings.removeAll(removes);
		return this;
	}

	private List<StandardEntity> calcInFireCluster(int currentTime,List<StandardEntity> agents){
		List<Collection<StandardEntity>> clusterList = getAllClusterEntities();
		int allClusterNumber = 0;//全部のクラスターに含まれている建物の数
		ArrayList<Double> rateList = new ArrayList<>();

		//小さなクラスタ(初期発火)を優先
		mostValuable m = new mostValuable();
		mostValuableInfo(currentTime,m,agents,clusterList);

		agents.clear();
		agents.addAll(m.getAgents());
		clusterList.clear();
		clusterList.addAll(m.getList());
		
		int agentNumber = agents.size();
		
		for(Collection<StandardEntity> list:clusterList){
			allClusterNumber+=list.size();
		}
		if(allClusterNumber!=0){
			for(Collection<StandardEntity> list:clusterList){
				double rate = list.size()/allClusterNumber;
				rateList.add(rate);
			}
		}
		if(!rateList.isEmpty()){
			
			for(int i=0;i<clusterList.size();i++){
				double rate = rateList.get(i);
				int fbNumber = (int)(rate*agentNumber);
				int index = 0;
				int count = 0;
				for(StandardEntity se:clusterList.get(i)){
					index = fireClustering.getClusterIndex(se);
					break;
				}

				for(EntityID target:fireClustering.getClusterEntityIDs(index)){
					if (agents.size() > 0 && count<fbNumber) {
						StandardEntity targetEntity = this.worldInfo.getEntity(target);
						if (targetEntity != null) {
							agents.sort(new DistanceSorter(this.worldInfo, targetEntity));
							StandardEntity result = agents.get(0);
							agents.remove(0);
							FireBrigadeInfo info = this.agentInfoMap.get(result.getID());
							if (info != null) {
								info.canNewAction = false;
								info.target = target;
								info.commandTime = currentTime;
								this.agentInfoMap.put(result.getID(), info);
								count++;
							}
						}
					}
				}
			}
		}
		return agents;
	}

	//innter
	class mostValuable{
		List<StandardEntity> agents;
		List<Collection<StandardEntity>> list;

		public mostValuable() {
			agents = new ArrayList<>();
			list = new ArrayList<>();
		}

		void setList(List<StandardEntity> a,List<Collection<StandardEntity>> l) {
			if(!agents.isEmpty())
				this.agents.clear();
			if(!list.isEmpty())
				this.list.clear();

			this.agents.addAll(a);
			this.list.addAll(l);

		}

		List<StandardEntity> getAgents() {
			return this.agents;
		}

		List<Collection<StandardEntity>> getList(){
			return this.list;
		}

	}

	private void mostValuableInfo(int currentTime,mostValuable m,List<StandardEntity> agents,List<Collection<StandardEntity>> clusterList) {
		double rate = 1/2;
		int fbNumber = (int)(rate*agents.size());
		List<StandardEntity> removedAgents =  new ArrayList<>();
		EntityID target = null;
		int removedIndex = -1;


		for(int i=0;i<clusterList.size();i++){

			int index = 0;
			int count = 0;


			if(clusterList.get(i).size()<3) {
				removedIndex = i;
				//System.out.println("mostValuableInfo");
				for(StandardEntity se:clusterList.get(i)){
					index = fireClustering.getClusterIndex(se);
					break;
				}


				for(EntityID entityID:fireClustering.getClusterEntityIDs(index)){
					StandardEntity se = this.worldInfo.getEntity(entityID);
					if(se instanceof Building) {
						Building building = (Building)se;
						if(building.getFieryness()==1) {
							target=entityID;
							break;
						}
					}
				}
				if(target==null) {
					for(EntityID entityID:fireClustering.getClusterEntityIDs(index)){
						target=entityID;
						break;
					}
				}

				for(StandardEntity agent:agents) {
					if (agents.size() > 0 && count<fbNumber) {
						StandardEntity targetEntity = this.worldInfo.getEntity(target);
						if (targetEntity != null) {

							StandardEntity result = agent;
							//agents.remove(0);
							FireBrigadeInfo info = this.agentInfoMap.get(result.getID());
							if (info != null) {
								info.canNewAction = false;
								info.target = target;
								info.commandTime = currentTime;
								this.agentInfoMap.put(result.getID(), info);
								count++;
								removedAgents.add(result);
							}
						}
					}else {
						break;
					}
				}
				if(!removedAgents.isEmpty()) {
					agents.removeAll(removedAgents);
				}
				

				break;
			}
		}
		if(removedIndex>-1) {
			clusterList.remove(removedIndex);
		}
		m.setList(agents, clusterList);

	}

	private List<Collection<StandardEntity>> getAllClusterEntities() {
		int number = fireClustering.getClusterNumber();
		List<Collection<StandardEntity>> result = new ArrayList<>(number);
		for(int i = 0; i < number; i++) {
			result.add(i, fireClustering.getClusterEntities(i));
		}
		return result;
	}

	@Override
	public FireTargetAllocator updateInfo(MessageManager messageManager) {
		fireClustering.updateInfo(messageManager);
		super.updateInfo(messageManager);
		if (this.getCountUpdateInfo() >= 2) {
			return this;
		}
		int currentTime = this.agentInfo.getTime();
		//受け取ったメッセージのBuildingが燃えてたら，targetBuildingsにadd
		//燃えてなかったら，priorityBuildings，targetBuildingsからremove
		for (CommunicationMessage message : messageManager.getReceivedMessageList(MessageBuilding.class)) {
			MessageBuilding mb = (MessageBuilding) message;
			Building building = MessageUtil.reflectMessage(this.worldInfo, mb);
			if (building.isOnFire()) {
				this.targetBuildings.add(building.getID());
			} else {
				this.priorityBuildings.remove(mb.getBuildingID());
				this.targetBuildings.remove(mb.getBuildingID());
			}
		}

		for (CommunicationMessage message : messageManager.getReceivedMessageList(MessageFireBrigade.class)) {
			MessageFireBrigade mfb = (MessageFireBrigade) message;
			MessageUtil.reflectMessage(this.worldInfo, mfb);
			FireBrigadeInfo info = this.agentInfoMap.get(mfb.getAgentID());
			if (info == null) {
				info = new FireBrigadeInfo(mfb.getAgentID());
			}
			if (currentTime >= info.commandTime + 2) {
				this.agentInfoMap.put(mfb.getAgentID(), this.update(info, mfb));
			}
		}
		for (CommunicationMessage message : messageManager.getReceivedMessageList(CommandFire.class)) {
			CommandFire command = (CommandFire) message;
			if (command.getAction() == CommandFire.ACTION_EXTINGUISH && command.isBroadcast()) {
				this.priorityBuildings.add(command.getTargetID());
				this.targetBuildings.add(command.getTargetID());
			}
		}
		for (CommunicationMessage message : messageManager.getReceivedMessageList(MessageReport.class)) {
			MessageReport report = (MessageReport) message;
			FireBrigadeInfo info = this.agentInfoMap.get(report.getSenderID());
			if (info != null && report.isDone()) {
				info.canNewAction = true;
				this.priorityBuildings.remove(info.target);
				this.targetBuildings.remove(info.target);
				info.target = null;
				this.agentInfoMap.put(report.getSenderID(), info);
			}
		}
		return this;
	}

	private Map<EntityID, EntityID> convert(Map<EntityID, FireBrigadeInfo> infoMap) {
		Map<EntityID, EntityID> result = new HashMap<>();
		for (EntityID id : infoMap.keySet()) {
			FireBrigadeInfo info = infoMap.get(id);
			if (info != null && info.target != null) {
				result.put(id, info.target);
			}
		}
		return result;
	}

	private List<StandardEntity> getActionAgents(Map<EntityID, FireBrigadeInfo> infoMap) {
		List<StandardEntity> result = new ArrayList<>();
		int agentsNumber = this.worldInfo.getEntitiesOfType(StandardEntityURN.FIRE_BRIGADE).size();
		double useNumber = agentsNumber*0.8;//全体の八割のエージェントに命令する
		int count = 0;
		for (StandardEntity entity : this.worldInfo.getEntitiesOfType(StandardEntityURN.FIRE_BRIGADE)) {
			if(count>useNumber){
				break;//8割に越えたら終わる
			}
			FireBrigadeInfo info = infoMap.get(entity.getID());
			if (info != null && info.canNewAction && ((FireBrigade) entity).isPositionDefined() ) {
				result.add(entity);
				count++;
			}
		}
		return result;
	}

	private FireBrigadeInfo update(FireBrigadeInfo info, MessageFireBrigade message) {
		if (message.isBuriednessDefined() && message.getBuriedness() > 0) {
			info.canNewAction = false;
			if (info.target != null) {
				this.targetBuildings.add(info.target);
				info.target = null;
			}
			return info;
		}
		if (message.getAction() == MessageFireBrigade.ACTION_REST) {
			info.canNewAction = true;
			if (info.target != null) {
				this.targetBuildings.add(info.target);
				info.target = null;
			}
		} else if (message.getAction() == MessageFireBrigade.ACTION_REFILL) {
			info.canNewAction = (message.getWater() + this.maxPower >= this.maxWater);
			if (info.target != null) {
				this.targetBuildings.add(info.target);
				info.target = null;
			}
		} else if (message.getAction() == MessageFireBrigade.ACTION_MOVE) {
			if (message.getTargetID() != null) {
				StandardEntity entity = this.worldInfo.getEntity(message.getTargetID());
				if (entity != null && entity instanceof Area) {
					if (info.target != null) {
						StandardEntity targetEntity = this.worldInfo.getEntity(info.target);
						if (targetEntity != null && targetEntity instanceof Area) {
							if (message.getTargetID().getValue() == info.target.getValue()) {
								info.canNewAction = false;
							} else {
								info.canNewAction = true;
								this.targetBuildings.add(info.target);
								info.target = null;
							}
						} else {
							info.canNewAction = true;
							info.target = null;
						}
					} else {
						info.canNewAction = true;
					}
				} else {
					info.canNewAction = true;
					if (info.target != null) {
						this.targetBuildings.add(info.target);
						info.target = null;
					}
				}
			} else {
				info.canNewAction = true;
				if (info.target != null) {
					this.targetBuildings.add(info.target);
					info.target = null;
				}
			}
		} else if (message.getAction() == MessageFireBrigade.ACTION_EXTINGUISH) {
			info.canNewAction = true;
			info.target = null;
			this.priorityBuildings.remove(message.getTargetID());
			this.targetBuildings.remove(message.getTargetID());
		}
		return info;
	}

	private class FireBrigadeInfo {
		EntityID agentID;
		EntityID target;
		boolean canNewAction;
		int commandTime;

		FireBrigadeInfo(EntityID id) {
			agentID = id;
			target = null;
			canNewAction = true;
			commandTime = -1;
		}
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
