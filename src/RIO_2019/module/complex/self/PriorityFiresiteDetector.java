package RIO_2019.module.complex.self;

import adf.agent.action.Action;
import adf.agent.action.ambulance.ActionRescue;
import adf.agent.action.common.ActionMove;
import adf.agent.communication.MessageManager;
import adf.agent.communication.standard.bundle.MessageUtil;
import adf.agent.communication.standard.bundle.StandardMessagePriority;
import adf.agent.communication.standard.bundle.information.*;
import adf.agent.communication.standard.bundle.centralized.CommandFire;
import adf.agent.communication.standard.bundle.centralized.CommandPolice;
import adf.agent.develop.DevelopData;
import adf.agent.info.AgentInfo;
import adf.agent.info.ScenarioInfo;
import adf.agent.info.WorldInfo;
import adf.agent.module.ModuleManager;
import adf.agent.precompute.PrecomputeData;
import adf.component.communication.CommunicationMessage;
import adf.component.module.algorithm.Clustering;
import adf.component.module.algorithm.PathPlanning;
import adf.component.module.complex.HumanDetector;
import misc.HumanAttributes;
import rescuecore2.misc.Pair;
import rescuecore2.misc.geometry.Point2D;
import rescuecore2.misc.geometry.Vector2D;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;

import java.awt.Polygon;
import java.awt.Shape;
import java.util.*;

//import RIO_2019.module.complex.self.teamDetector.DistanceSorter;

import static rescuecore2.standard.entities.StandardEntityURN.*;

public class PriorityFiresiteDetector extends HumanDetector {

	private Clustering clustering;

	private EntityID result;
	private EntityID previousResult = null;
	private ArrayList<AmbulanceTeam> activeAT;
	private ArrayList<Human> targets;
	private ArrayList<EntityID> ATGroup;
	private ArrayList<StandardEntity> actAT;

	private int sendTime;
	private int sendingAvoidTimeClearRequest;
	// 試作
	private PathPlanning pathPlanning;
	// ここまで
	// flag & communication
	private boolean isRadio = true;
	private int channelMax = 0;
	int voice = 256;
	int voiceCount = 1;
	
	//sendRequest
	private EntityID lastPosition;
	private int lastSentTime;
	private int stayCount;
	private int maxTimeStep = Integer.MAX_VALUE;
	private int estimatedMoveDistance;

	// CommandHistory(前の行動を参照する)
	HashMap<Integer, Pair<Integer, Integer>> PositionHistory;
	private ArrayList<EntityID> tempBlackList;

	public PriorityFiresiteDetector(AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager,
			DevelopData developData) {
		super(ai, wi, si, moduleManager, developData);

		this.result = null;
		this.sendTime = 0;
		this.sendingAvoidTimeClearRequest = developData.getInteger("SampleHumanDetector.sendingAvoidTimeClearRequest",
				5);
		this.activeAT = new ArrayList<>();
		this.targets = new ArrayList<>();
		this.ATGroup = new ArrayList<>();
		this.actAT = new ArrayList<>();
		this.tempBlackList = new ArrayList<>();

		this.PositionHistory = new HashMap<>();

		this.clustering = moduleManager.getModule("HumanDetector.Clustering", "RIO_2019.module.algorithm.RioneKmeansPP");

		this.channelMax = this.scenarioInfo.getCommsChannelsCount();
		if (channelMax < 2)
			isRadio = false; // 最大チャンネル数が2以下で通信不可
		// 試作
		this.pathPlanning = moduleManager.getModule("ActionTransport.PathPlanning",
				"RIO_2019.module.algorithm.AstarPathPlanning");
		// ここまで
		
		//sendRequest
		this.lastPosition = new EntityID(0);
		this.lastSentTime = 0;
		this.stayCount = 0;
		this.sendingAvoidTimeClearRequest = developData.getInteger("sample.tactics.MessageTool.sendingAvoidTimeClearRequest", 5);
		this.estimatedMoveDistance = developData.getInteger("sample.tactics.MessageTool.estimatedMoveDistance", 40000);
	}

	@Override
	public HumanDetector updateInfo(MessageManager messageManager) {
		super.updateInfo(messageManager);
		if (this.getCountUpdateInfo() >= 2) {
			return this;
		}
		this.clustering.updateInfo(messageManager);

		// 帯域の制限
		if (channelMax >= 2)
			isRadio = true;

		Set<EntityID> changedEntities = this.worldInfo.getChanged().getChangedEntities();
		changedEntities.add(this.agentInfo.me().getID());

		List<Building> burningBuildings = new ArrayList<Building>();

		ArrayList<AmbulanceTeam> workingAT = new ArrayList<>();
		ArrayList<Human> workedtargets = new ArrayList<>();
		ArrayList<Human> humansInBlockade = new ArrayList<>();

		// 視界情報の更新
		changedEntities.add(this.agentInfo.me().getID());
		// 自分の視界情報を送信
		for (EntityID id : changedEntities) {
			StandardEntity standardEntity = this.worldInfo.getEntity(id);

			if (standardEntity instanceof Civilian) {
				Civilian civilian = (Civilian) standardEntity;
				StandardEntity position = this.worldInfo.getEntity(civilian.getPosition());
				StandardEntityURN positionURN = position.getStandardURN();
				if (positionURN != REFUGE) {
					if (civilian.isHPDefined() && civilian.getHP() > 0) {
						if ((civilian.isBuriednessDefined() && civilian.getBuriedness() > 0)
								|| (civilian.isDamageDefined() && civilian.getDamage() > 0)) {
							messageManager.addMessage(new MessageCivilian(true, civilian));
						}
					}
				}

			} else if (standardEntity instanceof Building) {
				Building building = (Building) standardEntity;
				if (building.isOnFire()) {
					messageManager.addMessage(new MessageBuilding(true, building));
					burningBuildings.add(building);
				}
			}
		}
		// FBに消火命令を送信（できているか不明）
		if (burningBuildings.size() > 0) {
			messageManager.addMessage(
					new CommandFire(isRadio, null, burningBuildings.get(0).getID(), CommandFire.ACTION_EXTINGUISH));
		}
		
		sendExtendedRequestMessages(messageManager);

		// 他Agentが送信した情報を受信して処理
		for (CommunicationMessage message : messageManager.getReceivedMessageList()) {
			Class<? extends CommunicationMessage> messageClass = message.getClass();
			if (messageClass == MessageCivilian.class) {
				MessageCivilian mc = (MessageCivilian) message;
				if (!changedEntities.contains(mc.getAgentID())) {
					MessageUtil.reflectMessage(this.worldInfo, mc);
				}
			} else if (messageClass == MessageBuilding.class) {
				MessageBuilding mb = (MessageBuilding) message;
				if (!changedEntities.contains(mb.getBuildingID())) {
					MessageUtil.reflectMessage(this.worldInfo, mb);
				}
			} else if (messageClass == MessageAmbulanceTeam.class) {
				MessageAmbulanceTeam mat = (MessageAmbulanceTeam) message;
				if (!changedEntities.contains(mat.getAgentID())) {
					MessageUtil.reflectMessage(this.worldInfo, mat);
				}
				if (mat.getAction() == MessageAmbulanceTeam.ACTION_RESCUE
						|| mat.getAction() == MessageAmbulanceTeam.ACTION_LOAD) {
					if (mat.getAgentID() != this.agentInfo.me().getID()) {
						workingAT.add((AmbulanceTeam) this.worldInfo.getEntity(mat.getAgentID()));
						workedtargets.add((Human) this.worldInfo.getEntity(mat.getTargetID()));
					}
				} else if (mat.getAction() == MessageAmbulanceTeam.ACTION_MOVE) {
					if (mat.getAgentID() != this.agentInfo.me().getID()) {
						StandardEntity standardEntity = this.worldInfo.getEntity(mat.getTargetID());
						if (standardEntity.getStandardURN() != REFUGE) {
							workingAT.add((AmbulanceTeam) this.worldInfo.getEntity(mat.getAgentID()));
						}
					}
				}
			} else if (messageClass == MessageFireBrigade.class) {
				MessageFireBrigade mfb = (MessageFireBrigade) message;
				if (!changedEntities.contains(mfb.getAgentID())) {
					MessageUtil.reflectMessage(this.worldInfo, mfb);
				}
			} else if (messageClass == MessagePoliceForce.class) {
				MessagePoliceForce mpf = (MessagePoliceForce) message;
				if (!changedEntities.contains(mpf.getAgentID())) {
					MessageUtil.reflectMessage(this.worldInfo, mpf);
				}
			}
		}

		// 瓦礫に埋まっているHumanを集める
		for (StandardEntity agent : this.worldInfo.getEntitiesOfType(FIRE_BRIGADE, AMBULANCE_TEAM, CIVILIAN)) {
			if (agent instanceof Human && ((Human) agent).isXDefined() && ((Human) agent).isYDefined()) {
				Human human = (Human) agent;
				int agentX1 = human.getX();
				int agentY1 = human.getY();
				StandardEntity positionEntity = this.worldInfo.getPosition(human);
				if (positionEntity instanceof Road) {
					Road road = (Road) positionEntity;
					if (road.isBlockadesDefined() && road.getBlockades().size() > 0) {
						for (Blockade blockade : this.worldInfo.getBlockades(road)) {
							if (!blockade.isApexesDefined()) {
								continue;
							} // 瓦礫があったら:isInside,瓦礫に挟まっているかを判定
							if (this.isInside(agentX1, agentY1, blockade.getApexes())) {
								humansInBlockade.add(human);
							}
						}
					}
				}
			}
		}
		// targetの絞り込み
		this.activeAT.clear();
		this.targets.clear();

		int clusterIndex = clustering.getClusterIndex(this.agentInfo.me().getID());
		Collection<StandardEntity> elements = clustering.getClusterEntities(clusterIndex);

		for (StandardEntity next : this.worldInfo.getEntitiesOfType(AMBULANCE_TEAM)) {
			Human h = (Human) next;
			StandardEntity positionEntity = this.worldInfo.getPosition(h);
			if (positionEntity != null && positionEntity instanceof Building) {
				if (elements.contains(positionEntity) || elements.contains(h)) {
					if (h.isHPDefined() && h.getHP() > 0) {
						if (h.isBuriednessDefined() && h.getBuriedness() > 0) {
							if (!isONfire(h.getID()) && !isEntranceblock(h.getID()))
								targets.add(h);
						}
					}
				}
			} else if (positionEntity != null && positionEntity instanceof Area) {
				AmbulanceTeam at = (AmbulanceTeam) next;
				activeAT.add(at);
			}
		}

		for (StandardEntity next : this.worldInfo.getEntitiesOfType(FIRE_BRIGADE, POLICE_FORCE)) {
			Human h = (Human) next;
			StandardEntity positionEntity = this.worldInfo.getPosition(h);
			if (positionEntity != null) {
				if (elements.contains(positionEntity) || elements.contains(h)) {
					if (positionEntity instanceof Building) {
						if (h.isHPDefined() && h.isBuriednessDefined() && h.getHP() > 0 && h.getBuriedness() > 0) {
							if (!isONfire(h.getID()) && !isEntranceblock(h.getID()))
								targets.add(h);
						}
					}
				}
			}
		}
		for (StandardEntity next : this.worldInfo.getEntitiesOfType(CIVILIAN)) {
			Human h = (Human) next;
			StandardEntity positionEntity = this.worldInfo.getPosition(h);
			StandardEntityURN positionURN = positionEntity.getStandardURN();
			if (positionEntity != null && positionEntity instanceof Area) {
				if (elements.contains(positionEntity) || elements.contains(h)) {
					if (h.isHPDefined() && h.getHP() > 0) {
						if ((h.isBuriednessDefined() && h.getBuriedness() > 0)
								|| (h.isDamageDefined() && h.getDamage() > 0) && (positionURN != REFUGE)
										&& positionURN != AMBULANCE_TEAM) {
							if (!isONfire(h.getID()) && !isEntranceblock(h.getID()))
								targets.add(h);
						}
					}
				}
			}
		}
		this.activeAT.removeAll(workingAT);
		this.activeAT.removeAll(humansInBlockade);
		this.targets.removeAll(workedtargets);
		this.targets.removeAll(humansInBlockade);

		// 瓦礫除去要求
		int currentTime = this.agentInfo.getTime();
		Human agent = (Human) this.agentInfo.me();
		int agentX = agent.getX();
		int agentY = agent.getY();
		StandardEntity positionEntity = this.worldInfo.getPosition(agent);
		if (positionEntity instanceof Road) {
			Road road = (Road) positionEntity;
			if (road.isBlockadesDefined() && road.getBlockades().size() > 0) {
				for (Blockade blockade : this.worldInfo.getBlockades(road)) {
					if (blockade == null || !blockade.isApexesDefined()) {
						continue;
					}
					if (this.isInside(agentX, agentY, blockade.getApexes())) {
						if ((this.sendTime + this.sendingAvoidTimeClearRequest) <= currentTime) {
							this.sendTime = currentTime;
							messageManager.addMessage(
									new CommandPolice(true, null, agent.getPosition(), CommandPolice.ACTION_CLEAR));
							break;
						}
					}
				}
			}
		}
		// 消火要求
		if (this.result != null) {
			StandardEntity entity = this.worldInfo.getEntity(this.result);
			if (entity != null && entity instanceof Human) {
				Human h = (Human) entity;
				if (h.isPositionDefined()) {
					StandardEntity humanPosition = this.worldInfo.getPosition(h);
					if (humanPosition != null) {
						if (humanPosition instanceof Building && ((Building) humanPosition).isOnFire()) {
							messageManager.addMessage(
									new CommandFire(true, null, humanPosition.getID(), CommandFire.ACTION_EXTINGUISH));
						} else if (humanPosition.getStandardURN() == AMBULANCE_TEAM && h.getStandardURN() == CIVILIAN) {
							messageManager.addMessage(new MessageCivilian(true, (Civilian) h));
						}
					}
				}
			}
		}
		// ATGroupが形成されている時targetの埋没度が0なら一人残して（load目的）他は解散
		if (this.result != null && this.ATGroup.size() > 0) {
			Human target = (Human) this.worldInfo.getEntity(this.result);
			if (target.isBuriednessDefined() && target.getBuriedness() == 0) {
				if (ATGroup.get(0) != this.agentInfo.me().getID()) {
					this.result = null; // 運ばないからtargetを引き継がない
				}
			}
		}
		previousResult = this.result;// これから運ぶEntity
		return this;
	}
	
	private void sendExtendedRequestMessages (MessageManager messageManager)
	{
		this. lastPosition = agentInfo.getPosition();
		if (agentInfo.me().getStandardURN() == AMBULANCE_TEAM
				|| agentInfo.me().getStandardURN() == FIRE_BRIGADE)
		{
			int currentTime = agentInfo.getTime();
			Human agent = (Human) agentInfo.me();
			int agentX = agent.getX();
			int agentY = agent.getY();
			StandardEntity positionEntity = worldInfo.getPosition(agent);
			if (positionEntity instanceof Road)
			{
				boolean isSendRequest = false;

				Road road = (Road) positionEntity;
				if (road.isBlockadesDefined() && road.getBlockades().size() > 0)
				{
					for (Blockade blockade : worldInfo.getBlockades(road))
					{
						if (blockade == null || !blockade.isApexesDefined())
						{ continue; }

						if (this.isInside(agentX, agentY, blockade.getApexes()))
						{ isSendRequest = true; }
					}
				}

				if (this.lastPosition != null && this.lastPosition.getValue() == road.getID().getValue())
				{
					this.stayCount++;
					if (this.stayCount > this.getMaxTravelTime(road))
					{
						isSendRequest = true;
					}
				}
				else
				{
					this.lastPosition = road.getID();
					this.stayCount = 0;
				}

				if (isSendRequest)
				{
					this.lastSentTime = currentTime;
					return;
				}
			}

			if (isInExtendedBlockade(agent) && ((currentTime - this.lastSentTime) >= this.sendingAvoidTimeClearRequest)){
				this.lastSentTime = currentTime;
				if(this.worldInfo.getEntity(agent.getPosition()) instanceof Road) {
					messageManager.addMessage(
							new CommandPolice( true, null, agent.getPosition(), CommandPolice.ACTION_CLEAR )
							);
				}else if(this.worldInfo.getEntity(agent.getPosition()) instanceof Building) {
					Building building = (Building)this.worldInfo.getEntity(agent.getPosition());
					for(EntityID entityID : building.getNeighbours()) {
						if(this.worldInfo.getEntity(agent.getPosition()) instanceof Road) {
							Road road =(Road)this.worldInfo.getEntity(agent.getPosition());
							if(road.isBlockadesDefined()&&road.getBlockades().size()>0) {
								messageManager.addMessage(
										new CommandPolice( true, null, road.getID(), CommandPolice.ACTION_CLEAR )
										);
								return;
							}
						}
					}
				}
			}
		}
	}
	
	
	private int getMaxTravelTime(Area area)
    {
        int distance = 0;
        List<Edge> edges = new ArrayList<>();
        for (Edge edge : area.getEdges())
        {
            if (edge.isPassable())
            {
                edges.add(edge);
            }
        }
        if (edges.size() <= 1)
        {
            return this.maxTimeStep;
        }
        for (int i = 0; i < edges.size(); i++)
        {
            for (int j = 0; j < edges.size(); j++)
            {
                if (i != j)
                {
                    Edge edge1 = edges.get(i);
                    double midX1 = (edge1.getStartX() + edge1.getEndX()) / 2;
                    double midY1 = (edge1.getStartY() + edge1.getEndY()) / 2;
                    Edge edge2 = edges.get(j);
                    double midX2 = (edge2.getStartX() + edge2.getEndX()) / 2;
                    double midY2 = (edge2.getStartY() + edge2.getEndY()) / 2;
                    int d = this.getDistance(midX1, midY1, midX2, midY2);
                    if (distance < d)
                    {
                        distance = d;
                    }
                }
            }
        }

        if (distance > 0)
        {
            return 1 + (int)Math.ceil( distance / (double)this.estimatedMoveDistance);
        }

        return this.maxTimeStep;
    }
	
	 
	/**
	 * 指定したHumanが拡張した瓦礫に挟まっているならtrueを返す関数
	 * @param human
	 * @return true or false
	 * @author 岡島
	 */
	private boolean isInExtendedBlockade(Human human) {
		if(!human.isXDefined() || !human.isXDefined()) return false;
		int agentX = human.getX();
		int agentY = human.getY();
		StandardEntity positionEntity = this.worldInfo.getPosition(human);
		//TODO Building間のエントランスに対応させる
		if(positionEntity instanceof Road){
			Road road = (Road)positionEntity;
			if(road.isBlockadesDefined() && road.getBlockades().size() > 0){
				for(Blockade blockade : worldInfo.getBlockades(road)){
					Shape extendedShape = getExtendedShape(blockade);
					if(extendedShape != null && extendedShape.contains(agentX, agentY)){
						return true;
					}
				}
			}
		}
		return false;
	}
	private Shape getExtendedShape(Blockade blockade) {
		Shape shape = null;
		int extendingLength = 1000;
		if (shape == null) {
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
				xs[i] += (vectorX / magnitude) * extendingLength + 0.5;
				ys[i] += (vectorY / magnitude) * extendingLength + 0.5;
			}
			shape = new Polygon(xs, ys, count);
		}
		return shape;
	}
	

	@Override
	public HumanDetector calc() {
		// 後で使う
		PositionHistory.put(agentInfo.getTime(), this.worldInfo.getLocation(this.agentInfo.me()));
		// 現在Loadを行っている
		Human transportHuman = this.agentInfo.someoneOnBoard();
		if (transportHuman != null) {
			this.result = transportHuman.getID();
			return this;
		}
		// targetを変更するかの判断
		if (this.result != null) {
			Human me = (Human) agentInfo.me();
			if (me.getHP() < 2000) {
				return null;
			}
			Human target = (Human) this.worldInfo.getEntity(this.result);
			if (target != null) {
				if (!target.isHPDefined() || target.getHP() == 0) {
					this.result = null;
				} else if (!target.isPositionDefined()) {
					this.result = null;
				} else {
					if (isONfire(this.result) || isEntranceblock(this.result))
						this.result = null;
					StandardEntity position = this.worldInfo.getPosition(target);
					if (position != null) {
						StandardEntityURN positionURN = position.getStandardURN();
						if (positionURN == REFUGE || positionURN == AMBULANCE_TEAM) {
							this.result = null;
						}
					}
				}
			}
		}
		// actionのtargetを決めるべき
		if (this.result == null) {
			this.result = this.ATGroupBehavior(targets);
		}
		if (this.result == null) {
			this.result = this.calcGreedy(targets);
		}
		return this;

	}

	private EntityID ATGroupBehavior(ArrayList<Human> targets) {
		ArrayList<MatchingData> results = new ArrayList<MatchingData>();
		// 評価値付targets;
		for (int i = 0; i < targets.size(); i++) {
			Human human = targets.get(i);
			double triage = this.triage(human, this.agentInfo.getID());
			if (triage <= 0) {
				// results.add(i, new MatchingData(null, null, -1.0));
				continue;
			} else {
				results.add(new MatchingData(null, human.getID(), triage));
			}
		}
		// 優先樹に順にソート
		quick_sort(results, 0, results.size() - 1);

		// 集団行動
		for (int i = 0; i < results.size(); i++) {
			if (results.get(i).getTargetID() == null)
				continue;
			this.ATGroup.clear();
			this.actAT.clear();

			Collection<StandardEntity> TargetInRange = worldInfo.getObjectsInRange(results.get(i).getTargetID(), 50000);
			if (TargetInRange.contains(this.agentInfo.me())) {
				for (StandardEntity se : TargetInRange) {
					if (se instanceof AmbulanceTeam) {
						actAT.add(se);
					}
				}
				if (actAT.size() > 0) {
					for (int j = 0; j < 3 && j < actAT.size(); j++) {
						actAT.sort(new DistanceSorter(this.worldInfo,
								this.worldInfo.getEntity(results.get(i).getTargetID())));
						ATGroup.add(actAT.get(0).getID());
						actAT.remove(0);
					}
				}

				if (ATGroup.contains(this.agentInfo.me().getID()) && canMove(results.get(i).getTargetID())) {
					return results.get(i).getTargetID();
				}
			}
		}
		return null;
	}

	private EntityID calcGreedy(ArrayList<Human> targets) {
		ArrayList<MatchingData> results = new ArrayList<MatchingData>();

		// 評価値付targets;
		for (int i = 0; i < targets.size(); i++) {
			Human human = targets.get(i);
			double triage = this.triage(human, this.agentInfo.getID());
			if (triage <= 0) {
				// results.add(i, new MatchingData(null, null, -1.0));
				continue;
			} else {
				results.add(new MatchingData(null, human.getID(), triage));
			}
		}
		if (!results.isEmpty()) {
			// 優先樹に順にソート
			quick_sort(results, 0, results.size() - 1);

			EntityID targetID = results.get(0).getTargetID();
			results.remove(0);
			return targetID;
		}
		return null;
	}

	/***
	 * 評価値の決め方を要検証
	 * 
	 * @param human
	 * @param id
	 * @return
	 */
	private double triage(Human human, EntityID id) {
		int SurvivalTime = 0;
		int totalDistance = 0;
		Refuge refuge;
		Collection<StandardEntity> DangerZone;
		Collection<StandardEntity> NeedRescueZone;
		int flag = 0;
		// refugeが0のときの処理を追加

		if (human.isHPDefined() && human.isDamageDefined()) {
			int HP = human.getHP();
			int Damage = human.getDamage();
			if (Damage == 0) {
				return -1;
			}
			SurvivalTime = calcSurviveTime(HP, Damage);
		}
		// 対象への距離
		if (!human.isPositionDefined()) {
			return -1;
		}
		StandardEntity myposition = worldInfo.getPosition(id);
		pathPlanning.setFrom(myposition.getID());
		pathPlanning.setDestination(human.getID());
		List<EntityID> pathToVictim = pathPlanning.calc().getResult();
		if (pathToVictim != null) {
			totalDistance = calcDistance(pathToVictim);
		}
		// 対象からrefugeへの距離
		Collection<StandardEntity> refuges = this.worldInfo.getEntitiesOfType(StandardEntityURN.REFUGE);

		Refuge nearRefuge = (Refuge) Collections.min(refuges,
				new DistanceSorter(this.worldInfo, this.worldInfo.getEntity(human.getID())));
		refuge = nearRefuge;
		StandardEntity position = worldInfo.getPosition(human);
		pathPlanning.setFrom(position.getID());
		pathPlanning.setDestination(refuge.getID());
		List<EntityID> pathToRefuge = pathPlanning.calc().getResult();
		if (pathToRefuge != null) {
			totalDistance += calcDistance(pathToRefuge);
		}

		int RescueTime = totalDistance / 20000 + human.getBuriedness();// 集団を作れたかも付け加える
		int baseScore = SurvivalTime - RescueTime;
		if (baseScore > 0) {
			// 火災との距離
			DangerZone = worldInfo.getObjectsInRange(human.getID(), 80000);
			NeedRescueZone = worldInfo.getObjectsInRange(human.getID(), 120000);
			NeedRescueZone.removeAll(DangerZone);

			// 火災があれば間に合わない範囲
			for (StandardEntity e : DangerZone) {
				if (e instanceof Building) {
					Building building = (Building) e;
					if (building.isOnFire()) {
						return 0;
					}
				}
			}
			// 火災があれば優先して助けるべき範囲
			for (StandardEntity e : NeedRescueZone) {
				if (e instanceof Building) {
					Building building = (Building) e;
					if (building.isOnFire()) {
						flag = 1;
						break;
					}
				}
			}
			if (flag == 1) {
				return 4;
			} else {
				return 3;
			}
		} else {
			return -1;
		}
	}

	private void quick_sort(ArrayList<MatchingData> results, int left, int right) {
		if (left >= right) {
			return;
		}
		double p = results.get((left + right) / 2).getScore();
		int l = left, r = right;
		MatchingData temp;
		while (l <= r) {
			while (results.get(l).getScore() < p) {
				l++;
			}
			while (results.get(r).getScore() > p) {
				r--;
			}
			if (l <= r) {
				temp = results.get(l);
				results.add(l, results.get(r));
				results.add(r, temp);
				l++;
				r--;
			}
		}
		quick_sort(results, left, r); // ピボットより左側をクイックソート
		quick_sort(results, l, right); // ピボットより右側をクイックソート
	}

	private int calcSurviveTime(int hp, int damage) {
		int HP = hp;
		int Damage = damage;
		int i = 0;
		while (HP > 0) {
			if (i % 2 == 0) {
				Damage++;
			}
			HP -= Damage;
			i++;
		}
		return i;
	}

	private int calcDistance(List<EntityID> path) {
		int moveDistance = 0;
		int i;

		for (i = 0; i < path.size() - 1; i++) {
			EntityID path1 = path.get(i);
			EntityID path2 = path.get(i + 1);
			moveDistance += worldInfo.getDistance(path1, path2);
		}

		return moveDistance;
	}

	private class MatchingData implements Comparable<MatchingData> {

		private EntityID AT;
		private EntityID target;
		private double score;

		public MatchingData(EntityID AT, EntityID target, double score) {
			this.AT = AT;
			this.target = target;
			this.score = score;
		}

		protected EntityID getATID() {
			return this.AT;
		}

		protected EntityID getTargetID() {
			return this.target;
		}

		protected double getScore() {
			return this.score;
		}

		protected void setScore(double score) {
			this.score = score;
		}

		protected void diffScore(double score) {
			this.score -= score;
		}

		public int compareTo(MatchingData machingData) {
			if (this.score < machingData.score) {
				return -1;
			} else if (this.score > machingData.score) {
				return 1;
			} else {
				return 0;
			}
		}

	}

	@Override
	public EntityID getTarget() {

		return this.result;
	}

	@Override
	public HumanDetector precompute(PrecomputeData precomputeData) {

		super.precompute(precomputeData);
		if (this.getCountPrecompute() >= 2) {

			return this;
		}
		return this;
	}

	@Override
	public HumanDetector resume(PrecomputeData precomputeData) {
		super.resume(precomputeData);
		if (this.getCountResume() >= 2) {
			return this;
		}
		return this;
	}

	@Override
	public HumanDetector preparate() {
		super.preparate();
		if (this.getCountPreparate() >= 2) {
			return this;
		}
		return this;
	}

	private class DistanceSorter1 implements Comparator<StandardEntity> {
		private StandardEntity reference;
		private WorldInfo worldInfo;

		DistanceSorter1(WorldInfo wi, StandardEntity reference) {
			this.reference = reference;
			this.worldInfo = wi;
		}

		public int compare(StandardEntity a, StandardEntity b) {

			Human a1 = (Human) a;
			Human b1 = (Human) b;

			if (a instanceof Human & b instanceof Human) {

				if (a1.isBuriednessDefined() & b1.isBuriednessDefined())
					return a1.getBuriedness() - b1.getBuriedness();
			}

			if (a1.getBuriedness() == b1.getBuriedness()) {

				int d1 = this.worldInfo.getDistance(this.reference, a);
				int d2 = this.worldInfo.getDistance(this.reference, b);
				return d1 - d2;
			}
			int d1 = this.worldInfo.getDistance(this.reference, a);
			int d2 = this.worldInfo.getDistance(this.reference, b);
			return d1 - d2;
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

	// judge fire
	private boolean isONfire(EntityID targetid) {

		List<StandardEntity> burningList = new ArrayList<>(this.worldInfo.getFireBuildings());
		StandardEntity entity = this.worldInfo.getEntity(targetid);
		if (entity != null && entity instanceof Human) {
			Human h = (Human) entity;
			if (h.isPositionDefined()) {
				StandardEntity humanPosition = this.worldInfo.getPosition(h);
				if (humanPosition != null) {
					if (humanPosition instanceof Building && ((Building) humanPosition).isOnFire()) {
						return true;

					} else if (burningList.contains(humanPosition)) {
						return true;
					}
				}
			}
		} else if (entity != null && entity instanceof Building) {
			if (((Building) entity).isOnFire() || burningList.contains(entity))
				return true;
		}
		return false;

	}

	private boolean isEntranceblock(EntityID targetid) {
		StandardEntity entity3 = this.worldInfo.getEntity(targetid);
		if (entity3 != null && entity3 instanceof Human) {
			Human H = (Human) entity3;
			if (H.isPositionDefined()) {
				StandardEntity humanPosition = this.worldInfo.getPosition(H);
				if (humanPosition != null) {
					if (humanPosition instanceof Building) {
						for (EntityID neighbor : ((Building) humanPosition).getNeighbours()) {
							StandardEntity entity4 = this.worldInfo.getEntity(neighbor);
							if (entity4 != null && entity4 instanceof Road) {
								Road road = (Road) entity4;
								if (road.isBlockadesDefined() && road.getBlockades().size() > 0) {
									return true;
								}
							}
						}
					}
				}
			}
		} else if (entity3 != null && entity3 instanceof Building) {
			for (EntityID neighbor : ((Building) entity3).getNeighbours()) {
				StandardEntity entity4 = this.worldInfo.getEntity(neighbor);
				if (entity4 != null && entity4 instanceof Road) {
					Road road = (Road) entity4;
					if (road.isBlockadesDefined() && road.getBlockades().size() > 0) {
						return true;
					}
				}
			}
		}
		return false;
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

	private int getDistance(double fromX, double fromY, double toX, double toY) {
		double dx = toX - fromX;
		double dy = toY - fromY;
		return (int) Math.hypot(dx, dy);
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

	private boolean canMove(EntityID target) {
		if (agentInfo.getTime() - 2 > 0) {
			if (PositionHistory.get(agentInfo.getTime() - 1).equals(PositionHistory.get(agentInfo.getTime() - 2))) {
				StandardEntity positionEntity = this.worldInfo.getPosition(target);
				if (positionEntity instanceof Building) {
					return true;
				}
				// 位置移動していなかったら
				this.tempBlackList.add(target);
				return false; // tartget rank lower
			}
		}
		return true;
	}

	// Utils
	private boolean isInBlockade(Human human) {
		if (!human.isXDefined() || !human.isXDefined())
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
}
