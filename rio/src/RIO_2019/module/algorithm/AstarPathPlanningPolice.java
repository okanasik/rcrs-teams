package RIO_2019.module.algorithm;

import adf.agent.communication.MessageManager;
import adf.agent.develop.DevelopData;
import adf.agent.info.AgentInfo;
import adf.agent.info.ScenarioInfo;
import adf.agent.info.WorldInfo;
import adf.agent.module.ModuleManager;
import adf.agent.precompute.PrecomputeData;
import adf.component.module.algorithm.PathPlanning;
import rescuecore2.standard.entities.Area;
import rescuecore2.standard.entities.Blockade;
import rescuecore2.standard.entities.Road;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.worldmodel.EntityID;

import java.util.*;

/*
 * 作成者:林光希
 * 作成日時:2016/12/17
 * 使用Agent:AT,FB,PF
 * Astarアルゴリズムの実コストにclearcostを使用
 */
public class AstarPathPlanningPolice extends PathPlanning {
    private EntityID from;
    private Collection<EntityID> targets;
    private List<EntityID> result;

    public AstarPathPlanningPolice(AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager, DevelopData developData) {
        super(ai, wi, si, moduleManager, developData);
    }

    @Override
    public List<EntityID> getResult() {
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
    public PathPlanning calc() {
        List<EntityID> open = new LinkedList<>();
        List<EntityID> close = new LinkedList<>();
        Map<EntityID, Node> nodeMap = new HashMap<>();
     
        open.add(this.from);
        nodeMap.put(this.from, new Node(null, this.from));
        close.clear();
     
        while(true){
        	//openlistが空なら探索失敗
            if(open.isEmpty()){
                this.result = null;
                return this;
            }
     
            //openlistからスコアが最も小さなnodeを取り出す
            Node n = null;
            for(EntityID id : open){
                Node node = nodeMap.get(id);
     
                if(n == null){
                    n = node;
                }else if(node.getScore() < n.getScore()){
                    n = node;
                }
            }
     
            //取り出したnodeが目的地なら探索終了. 親nodeを辿りpathを生成する
            if (isGoal(n.getID(), this.targets)){
                List<EntityID> path = new LinkedList<>();
                while(n != null){
                    path.add(0, n.getID());
                    n = nodeMap.get(n.getParent());
                }    
                this.result = path;
                return this;
            }
            
            //取り出したnodeが目的地でなければcloselistに移す
            open.remove(n.getID());
            close.add(n.getID());
            
            //取り出したnodeのneighborのスコアを計算する
            List<EntityID> neighbours = n.getNeighbor();
            for (EntityID neighbour : neighbours){
                Node m = new Node(n, neighbour);  
                /*
                 * open,closeどちらにも含まれない場合はopenに追加
                 * openに含まれていて新しいスコアが元のスコアより小さい場合は新しいスコアに更新
                 * closeに含まれていて新しいスコアが元のスコアより小さい場合は新しいスコアに更新してopenに移す
                 */
                if(!open.contains(neighbour) && !close.contains(neighbour)){
                    open.add(m.getID());
                    nodeMap.put(neighbour, m);
                }else if(open.contains(neighbour) && m.getScore() < nodeMap.get(neighbour).getScore()){
                    nodeMap.put(neighbour, m);
                }else if(close.contains(neighbour) && m.getScore() < nodeMap.get(neighbour).getScore()){
                    nodeMap.put(neighbour, m);
                    close.remove(m.getID());
                    open.add(m.getID());
                }
            }
        }
    } 

    private boolean isGoal(EntityID e, Collection<EntityID> targets) {
        return targets.contains(e);
    }
    
    //pathplanningに使用するNodeのクラス
    private class Node {
        private EntityID id;
        private EntityID parent;
        private List<EntityID> neighbor; 
        private double cost;
        private double heuristic;
     
        public Node(Node parent, EntityID id) {
            this.id = id;    
            if(parent == null){
                this.cost = 0;
                this.parent = null;
            }else{
                this.parent = parent.getID();
                this.cost = parent.getCost() + worldInfo.getDistance(parent.getID(), id) + getClearCost(id);
            }
            this.neighbor = ((Area)worldInfo.getEntity(id)).getNeighbours();
            if(targets.size() == 0){
            	this.heuristic = 0;
            }else{
                this.heuristic = worldInfo.getDistance(id, targets.toArray(new EntityID[targets.size()])[0]);
            }
        }
     
        private EntityID getID() {
            return id;
        }
     
        private double getCost() {
            return cost;
        }
     
        private double getScore() {
            return cost + heuristic;
        }
     
        private EntityID getParent() {
            return this.parent;
        }
        
        private List<EntityID> getNeighbor() {
			return this.neighbor;
		}

        private double getClearCost(EntityID id){
        	StandardEntity standardEntity = worldInfo.getEntity(id);
        	//StandardEntity me = agentInfo.me();
        	if(standardEntity instanceof Road){
        		Road road = (Road)standardEntity;
        		if(road.isBlockadesDefined() && road.getBlockades().size() > 0){
        			double cost = 0;
        			for(Blockade blockade : worldInfo.getBlockades(road)){
        				if(blockade.isRepairCostDefined()){
        					cost += blockade.getRepairCost();
        				}
        			}
        			return cost;
        		}else{
        			return 0;
        		}
        	}else{
        		return 0;
        	}
        }
    }
}
