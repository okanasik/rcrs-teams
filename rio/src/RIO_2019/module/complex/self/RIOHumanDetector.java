package RIO_2019.module.complex.self;

import adf.agent.communication.MessageManager;
import adf.agent.communication.standard.bundle.MessageUtil;
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
import rescuecore2.misc.Pair;
import rescuecore2.misc.geometry.Point2D;
import rescuecore2.misc.geometry.Vector2D;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;

import java.util.*;

import static rescuecore2.standard.entities.StandardEntityURN.*;

public class RIOHumanDetector extends HumanDetector {

	private Clustering clustering;

	private EntityID result;
	private EntityID previousResult = null;
	private ArrayList<AmbulanceTeam> activeAT;
	private ArrayList<Human> targets;
	private ArrayList<EntityID> ATGroup;
	private ArrayList<StandardEntity> actAT;

	private int sendTime;
	private int sendingAvoidTimeClearRequest;

	private int moveDistance;
	private PathPlanning pathPlanning;
	// flag & communication
	private boolean isRadio = true;
	private int channelMax = 0;
	int voice = 256;
	int voiceCount = 1;

	// CommandHistory(前の行動を参照する)
	HashMap<Integer, Pair<Integer, Integer>> PositionHistory;
	private ArrayList<EntityID> tempBlackList;

	public RIOHumanDetector(AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager,
			DevelopData developData) {
		super(ai, wi, si, moduleManager, developData);

		this.result = null;
		this.sendingAvoidTimeClearRequest = developData.getInteger("SampleHumanDetector.sendingAvoidTimeClearRequest",
				5);
		this.activeAT = new ArrayList<>();
		this.targets = new ArrayList<>();
		this.ATGroup = new ArrayList<>();
		this.actAT = new ArrayList<>();
		this.tempBlackList = new ArrayList<>();

		this.moveDistance = developData.getInteger("SampleHumanDetector.moveDistance", 40000);

		this.PositionHistory = new HashMap<>();

		this.clustering = moduleManager.getModule("HumanDetector.Clustering", "RIO_2019.module.algorithm.RioneKmeansPP");
		this.pathPlanning = moduleManager.getModule("ActionTransport.PathPlanning",
				"RIO_2019.module.algorithm.AstarPathPlanning");

		registerModule(pathPlanning);
		this.channelMax = this.scenarioInfo.getCommsChannelsCount();
		if (channelMax < 2)
			isRadio = false; // 最大チャンネル数が2以下で通信不可
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

		// 視界情報の更新
		Set<EntityID> changedEntities = this.worldInfo.getChanged().getChangedEntities();
		changedEntities.add(this.agentInfo.me().getID());

		StandardEntity myEntity = this.agentInfo.me();
		StandardEntity nearPF = null;
		List<Building> burningBuildings = new ArrayList<Building>();

		// 都度for文を回さないように,changedEntitiesの参照を極力まとめる
		for (EntityID id : changedEntities) {
			StandardEntity entity = this.worldInfo.getEntity(id);

			// 視界内のPFを一人取得(一番最後のPF)
			if (entity instanceof PoliceForce) {
				nearPF = entity;
			}
			// 視界内の燃えている建物を取得
			else if (entity instanceof Building && ((Building) entity).isOnFire()) {
				burningBuildings.add((Building) entity);
			}
		}

		// FBに消火命令を送信（できているか不明）
		if (burningBuildings.size() > 0) {
			messageManager.addMessage(
					new CommandFire(isRadio, null, burningBuildings.get(0).getID(), CommandFire.ACTION_EXTINGUISH));
		}

		ArrayList<AmbulanceTeam> workingAT = new ArrayList<>();
		ArrayList<Human> workedtargets = new ArrayList<>();
		ArrayList<Human> humansInBlockade = new ArrayList<>();

		// 視界情報の更新
		//Set<EntityID> changedEntities = this.worldInfo.getChanged().getChangedEntities();
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
				}
			}
		}
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
						if (standardEntity.getStandardURN() == REFUGE) {
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

		// 瓦礫に挟まているかどうかの判定（RioneRoadDetectorより拝借）
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
			if (positionEntity instanceof Area) {
				if (elements.contains(positionEntity) || elements.contains(h)) {
					if (h.isHPDefined() && h.getHP() > 0) {
						if (h.isBuriednessDefined() && h.getBuriedness() > 0) {
							if (!isONfire(h.getID()) && !isEntranceblock(h.getID()))
								targets.add(h);
						} else {
							activeAT.add((AmbulanceTeam) next);
						}
					}
				}
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
			if(!h.isPositionDefined()){
				continue;
			}

			StandardEntity positionEntity = this.worldInfo.getPosition(h);
			StandardEntityURN positionURN = positionEntity.getStandardURN();
			if (positionEntity instanceof Area) {
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
		this.targets.removeAll(workedtargets);
		this.targets.removeAll(humansInBlockade);

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

		if (this.result != null) {
			StandardEntity entity = this.worldInfo.getEntity(this.result);
			if (entity instanceof Human) {
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
			if (target != null && target.isBuriednessDefined() && target.getBuriedness() == 0) {
				if (ATGroup.get(0) != this.agentInfo.me().getID()) {
					this.result = null; // 運ばないからtargetを引き継がない
				}
			}
		}
		previousResult = this.result;
		return this;
	}

	@Override
	public HumanDetector calc() {
		// 過去の行動を参照する
		PositionHistory.put(agentInfo.getTime(), this.worldInfo.getLocation(this.agentInfo.me()));

		ArrayList<StandardEntity> ambulanceTeams = new ArrayList<StandardEntity>();

		ambulanceTeams.addAll(worldInfo.getEntitiesOfType(AMBULANCE_TEAM));

		Human transportHuman = this.agentInfo.someoneOnBoard();
		if (transportHuman != null) {
			this.result = transportHuman.getID();
			return this;
		}
		if (this.result != null) {
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
		if (this.result == null && !targets.isEmpty()) {
			this.result = this.ATGroupBehavior(targets);
		}
		return this;
	}

	/*
	 * ＜予定＞ 火災に近い市民を救助してるATの元に行くべき
	 */
	private EntityID ATGroupBehavior(ArrayList<Human> targets) {
		ArrayList<MatchingData> results = new ArrayList<MatchingData>();
		ArrayList<MatchingData> temp = new ArrayList<MatchingData>();
		// ArrayList<StandardEntity> building = new ArrayList<StandardEntity>();
		ArrayList<Building> burn = new ArrayList<Building>();
		ArrayList<Building> buildings = new ArrayList<Building>();
		int clusterSize = 0;
		int Range = 0;
		int numofAT = 2;
		double ratioofFire = 0;

		// 燃えている建物
		for (StandardEntity element : worldInfo.getEntitiesOfType(BUILDING)) {
			if (element instanceof Building) {
				// building.add(element);
				Building a = (Building) element;
				if (a.isOnFire()) {
					burn.add(a);
				}
				buildings.add(a);
			}
		}

		if (previousResult != null) {
			EntityID targetPos = ((Human) this.worldInfo.getEntity(previousResult)).getPosition();
			if (targetPos != null && this.worldInfo.getEntity(targetPos) instanceof AmbulanceTeam) {
				// System.out.println(previousResult + " is on AT");
				this.tempBlackList.clear();
			}
		}

		// 評価値付targets;
		for (int i = 0; i < targets.size(); i++) {
			if (targets.get(i) == null)
				continue;
			Human human = targets.get(i);
			int triage = this.triage(human, this.agentInfo.getID());
			if (triage <= 0) {
				temp.add(new MatchingData(null, null, -1.0));
				continue;
			} else {
				results.add(new MatchingData(null, human.getID(), triage));
			}
		}
		// 昇順にソート
		quick_sort(results, 0, results.size() - 1);

		results.addAll(temp);

		// ATを集める範囲を決定。時間経過で増加
		int clusterIndex = clustering.getClusterIndex(this.agentInfo.me().getID());
		Collection<StandardEntity> elements = clustering.getClusterEntities(clusterIndex);

		for (StandardEntity a : elements) {
			if (a instanceof Building) {
				Building building = (Building) a;
				clusterSize += building.getGroundArea();
			}
		}

		//0.41... = sqrt(2)-1
		//the range extends to double radius area at end of sim.

		//Range = (int) (Math.sqrt(clusterSize/Math.PI)*(1+0.414213*agentInfo.getTime()/scenarioInfo.getKernelTimesteps()));
		Range = (int) (Math.sqrt(clusterSize/Math.PI)*(1+0.414213*agentInfo.getTime()/300));

		////////////////////////////////////////////////////////////////////////
		for (int i = 0; i < results.size(); i++) {
			if (results.get(i).getTargetID() == null)
				continue;
			this.ATGroup.clear();
			this.actAT.clear();

			if (this.tempBlackList.contains(results.get(i).getTargetID()))
				continue;

			Collection<StandardEntity> TargetInRange = worldInfo.getObjectsInRange(results.get(i).getTargetID(), Range);
			if (TargetInRange.contains(this.agentInfo.me())) {
				for (StandardEntity se : TargetInRange) {
					if (se instanceof AmbulanceTeam) {// 近くに存在しているだけではだめ。
						actAT.add(se);
					}
				}
				if (actAT.size() > 0) {
					// 集団の人数は時間経過で増加（火災の広がりや残り時間のことを考えて）
					ratioofFire = (burn.size() * 100) / buildings.size();
					if (ratioofFire < 33) {
						numofAT++;
					} else if (ratioofFire < 66) {
						numofAT += 2;
					} else {
						numofAT += 3;
					}

					numofAT += agentInfo.getTime() / 90;

					///////////////////////////////////////////////////////////////////

					for (int j = 0; j < numofAT && j < actAT.size(); j++) {
						actAT.sort(new DistanceSorter(this.worldInfo,
								this.worldInfo.getEntity(results.get(i).getTargetID())));
						ATGroup.add(actAT.get(0).getID());
						actAT.remove(0);
					}
				}

				if (ATGroup.contains(this.agentInfo.me().getID()) && canMove(results.get(i).getTargetID())) {
					// System.out.println("targetID="+
					// results.get(i).getTargetID()+"agentID="+this.agentInfo.getID());
					// System.out.println("集まれ〜");
					return results.get(i).getTargetID();
				}
			}

			// 火災から十分離れていたら1対1救助。最も近い人を救助
			// 既にATがいるのか判断してcountinue?
			// とても集団行動できているが救助後団子状になって探索できていない。
			/*
			 * System.out.println("ぼっちかも"); EntityID resultID =
			 * results.get(i).getTargetID(); if (worldInfo.getEntity(resultID)
			 * instanceof Human) { Human result = (Human)
			 * worldInfo.getEntity(resultID); for (StandardEntity at :
			 * worldInfo.getEntitiesOfType(AMBULANCE_TEAM)) { if
			 * (result.getPosition() == ((Human) at).getPosition()) {
			 * System.out.println("おるやん"); break; } else {
			 * System.out.println("ぼっち"); return results.get(i).getTargetID(); }
			 * } }
			 */
		}
		return null;
	}

	private void quick_sort(ArrayList<MatchingData> results, int left, int right) {
		if (left >= right) {
			return;
		}
		double p = results.get((left + right) / 2).getScore();
		int l = left, r = right;
		MatchingData temp;
		while (l <= r) {
			while (results.get(l).getScore() > p) {
				l++;
			}
			while (results.get(r).getScore() < p) {
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

	private EntityID calcGreedy(ArrayList<Human> targets) {
		HashMap<Integer, MatchingData> results = new HashMap<Integer, MatchingData>();
		Map<Building, Integer> croudMember = this.croudedPosition(targets);

		// 評価値付targets;
		for (int i = 0; i < targets.size(); i++) {
			if (targets.get(i) == null)
				continue;
			Human human = targets.get(i);
			int triage = this.triage(human, this.agentInfo.getID());
			if (triage <= 0) {
				continue;
			} else {
				results.put(i, new MatchingData(null, human.getID(), triage));
				StandardEntity se = this.worldInfo.getEntity(human.getPosition());
				if (se instanceof Building) {
					Building building = (Building) se;
					if (building.isOnFire()) {
						results.get(i).setScore(-1);
					}
				}
			}
		}

		if (results.size() > 0) {
			int index = results.keySet().stream().min(Comparator.naturalOrder()).get();
			return results.get(index).getTargetID();
		}
		return null;
	}

	private int triage(Human human, EntityID id) {
		Collection<StandardEntity> refuges = this.worldInfo.getEntitiesOfType(StandardEntityURN.REFUGE);
		if (refuges.isEmpty())
			return 0;
		int TotalCost = 0;// 救助に何サイクルかかるか
		int survivalTime = 0;// 何サイクル生存するか
		int coopAT = 1;// 一緒に救助しているAT

		ArrayList<Human> ATs = new ArrayList<>();
		EntityID myPos = ((Human) worldInfo.getEntity(id)).getPosition();

		Refuge refuge = (Refuge) Collections.min(refuges, new DistanceSorter(this.worldInfo, human));

		// refugeへの移動距離
		pathPlanning.setFrom(human.getPosition());
		pathPlanning.setDestination(refuge.getID());
		List<EntityID> pathToRefuge = pathPlanning.calc().getResult();
		if (pathToRefuge == null) {
			// System.out.println("パス無し1");
			return 0;
		}
		int DistanceToRefuge = calcPathBasedDistance(pathToRefuge);
		// targetへの移動距離
		pathPlanning.setFrom(myPos);
		pathPlanning.setDestination(human.getPosition());
		List<EntityID> pathToTarget = pathPlanning.calc().getResult();
		if (pathToTarget == null) {
			// System.out.println("パス無し2");
			return 0;
		}
		int DistanceToTarget = calcPathBasedDistance(pathToTarget);

		for (EntityID element : worldInfo.getEntityIDsOfType(AMBULANCE_TEAM)) {
			StandardEntity se = worldInfo.getEntity(element);
			if (se instanceof Human) {
				ATs.add((Human) se);
			}
		}
		Map<Building, Integer> ATMap = croudedPosition(ATs);
		StandardEntity position = worldInfo.getPosition(human);
		if (position instanceof Building) {
			Building temp = (Building) position;
			if (ATMap.containsKey(temp)) {
				coopAT += ATMap.get(temp);
			}
		}

		TotalCost = (DistanceToRefuge + DistanceToTarget) / 40000 + human.getBuriedness() / coopAT;

		survivalTime = calcDamage(human);

		return survivalTime - TotalCost;

	}

	private int calcDamage(Human human) {
		int HP = human.getHP();
		int survivalTime = 0;
		double damage = human.getDamage();
		// int currentTime = agentInfo.getTime();
		/*
		 * bury : 0.000035 fire : 0.00025 collapse : 0.00025
		 */
		double k = 0.000035;
		/*
		 * bury : 0.01 fire : 0.03 collapse : 0.01
		 */
		double l = 0.01;
		double n = 0.38;// 適当（適当って意味じゃなく）

		while (HP - damage > 0) {
			damage = damage + (k * damage * damage) + l + n;
			survivalTime++;
		}
		return survivalTime;
	}

	private Map<Building, Integer> croudedPosition(List<Human> list) {
		Map<Building, Integer> pMap = new HashMap<Building, Integer>();
		int k = 0;
		for (Human human : list) {// サーバーから埋まっている人を持ってきて建物いるかを判定何人くらいいるのか
			StandardEntity pos = worldInfo.getEntity(human.getPosition());
			if (pos instanceof Building) {
				Building posBuilding = (Building) pos;
				if (pMap.containsKey(posBuilding)) {// buildingの中にいるかを判定
					k = pMap.get(posBuilding) + 1;
				}
				pMap.put(posBuilding, k);

			}
		}
		return pMap;
	}

	private int calcPathBasedDistance(List<EntityID> path) {
		int moveDistance = 0;
		int i;
		for (i = 0; i < path.size() - 2; i++) {
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

		protected void setScore(double score) {
			this.score = score;
		}

		protected double getScore() {
			return score;
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
		StandardEntity targetEntity = this.worldInfo.getEntity(targetid);
		if (targetEntity instanceof Human) {
			Human H = (Human) targetEntity;
			if (H.isPositionDefined()) {
				StandardEntity humanPosition = this.worldInfo.getPosition(H);
				if (humanPosition instanceof Building) {
					for (EntityID neighbor : ((Building) humanPosition).getNeighbours()) {
						StandardEntity neighborEntity = this.worldInfo.getEntity(neighbor);
						if (neighborEntity instanceof Road) {
							Road road = (Road) neighborEntity;
							if (road.isBlockadesDefined() && road.getBlockades().size() > 0) {
								return true;
							}
						}
					}
				}
			}
		} else if (targetEntity != null && targetEntity instanceof Building) {
			for (EntityID neighbor : ((Building) targetEntity).getNeighbours()) {
				StandardEntity neighborEntity = this.worldInfo.getEntity(neighbor);
				if (neighborEntity != null && neighborEntity instanceof Road) {
					Road road = (Road) neighborEntity;
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

	private int getMaxTravelTime(Area area) {
		int distance = 0;
		List<Edge> edges = new ArrayList<>();
		for (Edge edge : area.getEdges()) {
			if (edge.isPassable()) {
				edges.add(edge);
			}
		}
		if (edges.size() <= 1) {
			return Integer.MAX_VALUE;
		}
		for (int i = 0; i < edges.size(); i++) {
			for (int j = 0; j < edges.size(); j++) {
				if (i != j) {
					Edge edge1 = edges.get(i);
					double midX1 = (edge1.getStartX() + edge1.getEndX()) / 2;
					double midY1 = (edge1.getStartY() + edge1.getEndY()) / 2;
					Edge edge2 = edges.get(j);
					double midX2 = (edge2.getStartX() + edge2.getEndX()) / 2;
					double midY2 = (edge2.getStartY() + edge2.getEndY()) / 2;
					int d = this.getDistance(midX1, midY1, midX2, midY2);
					if (distance < d) {
						distance = d;
					}
				}
			}
		}
		if (distance > 0) {
			return (distance / this.moveDistance) + ((distance % this.moveDistance) > 0 ? 1 : 0) + 1;
		}
		return Integer.MAX_VALUE;
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
}
