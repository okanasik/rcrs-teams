package RIO_2019.module.algorithm;

import adf.agent.communication.MessageManager;
import adf.agent.develop.DevelopData;
import adf.agent.info.AgentInfo;
import adf.agent.info.ScenarioInfo;
import adf.agent.info.WorldInfo;
import adf.agent.module.ModuleManager;
import adf.agent.precompute.PrecomputeData;
import adf.component.module.algorithm.PathPlanning;
import rescuecore2.misc.collections.LazyMap;
import rescuecore2.standard.entities.Area;
import rescuecore2.standard.entities.Building;
import rescuecore2.worldmodel.Entity;
import rescuecore2.worldmodel.EntityID;

import java.util.*;


import rescuecore2.standard.entities.Road;
import rescuecore2.standard.entities.StandardEntity;

public class KaitoPathPlanning extends PathPlanning { // 経路探索（幅優先探索）

	private Map<EntityID, Set<EntityID>> graph; // マップ全体のAreaをグラフ化したもの
	private Map<EntityID, Set<EntityID>> Crossgraph; //キー（交差点）に対して、隣接する交差点を取得

	private EntityID from;
	private Collection<EntityID> targets;
	private List<EntityID> result;

	private Map<EntityID, Integer> crossRoads = new HashMap<>(); // 交差点リスト roadとroadcount (交差点のIDとそのneighbourの数)
	private Map<EntityID, Integer> edgeRoads = new HashMap<>(); // edgeリスト
	//private List<Edge> edge = new LinkedList<>(); // edgeリスト AreaのEntityIDから接している交差点を取得できる

	//private int count = 0; // PathPlanningが呼び出された回数

	public KaitoPathPlanning(AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager, DevelopData developData) {
		super(ai, wi, si, moduleManager, developData);
		this.init();
	}

	private void init() { // 初期化 グラフ作成
		Map<EntityID, Set<EntityID>> neighbours = new LazyMap<EntityID, Set<EntityID>>() { // Mapでキーと値を設定（キーに対して、キーに隣接するエリアを値に設定）
			@Override
			public Set<EntityID> createValue() {
				return new HashSet<>();
			}
		};
		for(Entity next : this.worldInfo){
			if(next instanceof Area){
				Collection<EntityID> areaNeighbours = ((Area)next).getNeighbours(); // nextに隣接するAreaの集合を取得

				neighbours.get(next.getID()).addAll(areaNeighbours); // たぶんgraphでAreaのIDを指定すると、そこに隣接するAreaのリストが得られる
				// edge.add(new Edge(next, null, null));
			}
		}
		this.graph = neighbours;
	}

	@Override
	public List<EntityID> getResult() { // PathPlanningの結果(経路のリスト)を返す
		return this.result;
	}

	@Override
	public PathPlanning setFrom(EntityID id) {
		this.from = id;
		return this;
	}

	@Override
	public PathPlanning setDestination(Collection<EntityID> targets) {
		this.targets = targets;
		return this;
	}

	@Override
	public PathPlanning updateInfo(MessageManager messageManager) {
		super.updateInfo(messageManager);
		return this;
	}

	@Override
	public PathPlanning precompute(PrecomputeData precomputeData) {
		super.precompute(precomputeData);
		return this;
	}

	@Override
	public PathPlanning resume(PrecomputeData precomputeData) {
		super.resume(precomputeData);
		return this;
	}

	@Override
	public PathPlanning preparate() {
		super.preparate();
		return this;
	}

	@Override
	public PathPlanning calc(){
		MakeCrossRoad();
		BFS(this.from, this.targets);
		return this;
	}
	
	private void BFS(EntityID from, Collection<EntityID> goal) {
		List<EntityID> open = new LinkedList<>(); // キュー
		Map<EntityID, EntityID> ancestors = new HashMap<>(); //クローズドリスト

		open.add(from);
		EntityID next = null;
		boolean found = false;
		ancestors.put(from, from);

		do {
			next = open.remove(0);
			//System.out.println(next);
			if (isGoal(next, goal)) {
				found = true;
				break;
			}

			Collection<EntityID> neighbours = graph.get(next); // nextに隣接するAreaを取得
			//System.out.println(neighbours);
			if (neighbours.isEmpty()) {
				continue;
			}

			for (EntityID neighbour : neighbours) {
				if (isGoal(neighbour, targets)) {
					ancestors.put(neighbour, next);
					next = neighbour;
					found = true;
					break;
				} else {
					if (!ancestors.containsKey(neighbour)) {// neighbourがキーに登録されてないことを確認
						open.add(neighbour);
						ancestors.put(neighbour, next);
					}
				}
			}
		}while(!found && !open.isEmpty());

		if(!found){ // No path
			this.result = null; // 探索失敗
		}
		////////// ここまで道を探してる（経路はまだ作ってない）/////////////////////////////////////////////
		// Walk back from goal to this.from
		EntityID current = next; // 目的地
		LinkedList<EntityID> path = new LinkedList<>();
		LinkedList<EntityID> crossRoadPath = new LinkedList<>();

		do {
			if (crossRoads.containsKey(current)) {
				crossRoadPath.add(0, current);
			}
			path.add(0, current);
			current = ancestors.get(current);

			if (current == null) {
				throw new RuntimeException("Found a node with no ancestor! Something is broken.");
			}
		} while (current != from);
		//System.out.println(path);
		//System.out.println(crossRoadPath);
		//System.out.println("----------------");
		Road2NextIntersection(path);
		//System.out.println(Crossgraph.size());
		//System.out.println("--------------------------------");
		this.result = path; // pathは一番前が始点、後ろが目的地
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	private void MakeCrossRoad() {// 交差点リストを作る
		for(Entity next : this.worldInfo){
			if(next instanceof Area){
				EntityID next2 = next.getID();
				DetectCrossRoad(next2);
			}
		}
	}

	private void DetectCrossRoad(EntityID road1){ // road1が交差点ならリストに入れる(交差点のIDとそのneighbourの数)
		EntityID next = road1;
		int roadcount = 0;
		// 交差点条件 road1がRoad型であること、かつ接しているEdgeが３つか４つ。

		StandardEntity next2 = worldInfo.getEntity(next);
		if(next2 instanceof Road){
			Collection<EntityID> neighbour = graph.get(next);
			roadcount = 0;

			for(EntityID road : neighbour){
				if(isEdge(road)){
					roadcount++;
				}
			}

			if (roadcount > 2 && roadcount < 5) {
				// crossRoadsに交差点とroadcountを格納
				if (!crossRoads.containsKey(next)) {
					crossRoads.put(next, roadcount); // 交差点のIDと接している道の数
				}
			}else if(roadcount <= 2){
				if(!edgeRoads.containsKey(next)){
					edgeRoads.put(next, roadcount); // エッジのIDと接している道の数
				}
			}
		}
	}

	private boolean isEdge(EntityID road1){  // edge条件 road1がRoad型であること、かつ接しているエリア(Road型)（エントランスでない）が２つ以下。
		EntityID next = road1;
		int roadcount = 0;
		boolean isedge = false;

		StandardEntity next2 = worldInfo.getEntity(next);
		if(next2 instanceof Road){ // 1. road1がRoad型であること
			Collection<EntityID> neighbour = graph.get(next);
			roadcount = 0;

			for (EntityID road : neighbour) {
				StandardEntity road2 = worldInfo.getEntity(road);
				if (road2 instanceof Road) { // 2. nextに隣接するエリア(road2)がRoadの時
					roadcount++;
				}
			}
			if (roadcount > 0) {
				isedge = true;
				return isedge;
			}
		}
		return isedge;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	private void Road2NextIntersection(LinkedList<EntityID> path){
		Map<EntityID, Set<EntityID>> nextIntersection = new LazyMap<EntityID, Set<EntityID>>() { // Mapでキーと値を設定（キーに対して、キーに隣接するエリアを値に設定）
			@Override
			public Set<EntityID> createValue() {
				return new HashSet<>();
			}
		};
		LinkedList<EntityID> crossRoadPath = new LinkedList<>();
		
		int count = 0;
		
		for(EntityID road : path){
			if(crossRoads.containsKey(road)){
				crossRoadPath.add(road);
				//System.out.println(crossRoadPath);
				//System.out.println("start");
				//System.out.println(crossRoadPath.get(0));
				EntityID start = crossRoadPath.get(0);
				//System.out.println("end");
				//System.out.println(crossRoadPath.get(crossRoadPath.size()-1));
				EntityID nextCross = crossRoadPath.get(crossRoadPath.size()-1);
				nextIntersection.get(start).add(nextCross);
				
				crossRoadPath = new LinkedList<>();
				EntityID start_intersection = road;
				crossRoadPath.add(start_intersection);
				continue;
			}
			crossRoadPath.add(road);
		}
		this.Crossgraph = nextIntersection;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	private class Edge { // ２つの交差点とその間の経路を管理
		private Map<EntityID, Set<EntityID>> nearEdge = new LazyMap<EntityID, Set<EntityID>>() {
			@Override
			public Set<EntityID> createValue() {
				return new HashSet<>();
			}
		};
		private Collection<EntityID> nodes;

		private Edge(EntityID edge, Collection<EntityID> nodes) { // エリアのIDから２つの交差点を取得
			nearEdge.get(edge).addAll(nodes);
		}

		private Edge(EntityID edge, EntityID node1, EntityID node2) { // エリアのIDから２つの交差点を取得
			nodes = null;
			nodes.add(node1);
			nodes.add(node2);
			nearEdge.get(edge).addAll(nodes);
		}

		private Set<EntityID> getNode(EntityID edge) {
			return nearEdge.get(edge);
		}
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    
	private boolean isGoal(EntityID e, Collection<EntityID> test) {
		return test.contains(e);
	}

	private boolean isIntersection(EntityID road1, EntityID road2, Map<EntityID, Integer> intersections) {// (始点、目的の交差点、交差点リスト)
		if(!road2.equals(road1)){
			return intersections.containsKey(road2);
		}else{
			return false;
		}
	}
}

