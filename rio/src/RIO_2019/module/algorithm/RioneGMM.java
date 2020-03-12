package RIO_2019.module.algorithm;

import adf.agent.communication.MessageManager;
import adf.agent.develop.DevelopData;
import adf.agent.info.AgentInfo;
import adf.agent.info.ScenarioInfo;
import adf.agent.info.WorldInfo;
import adf.agent.module.ModuleManager;
import adf.agent.precompute.PrecomputeData;
import adf.component.module.algorithm.Clustering;
import adf.component.module.algorithm.StaticClustering;
import rescuecore2.misc.Pair;
import rescuecore2.misc.collections.LazyMap;
import rescuecore2.misc.geometry.Point2D;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.Entity;
import rescuecore2.worldmodel.EntityID;

import java.util.*;


public class RioneGMM extends StaticClustering {
    private static final String KEY_CLUSTER_SIZE = "sample.clustering.size";
    private static final String KEY_CLUSTER_CENTER = "sample.clustering.centers";
    private static final String KEY_CLUSTER_ENTITY = "sample.clustering.entities.";
    private static final String KEY_ASSIGN_AGENT = "sample.clustering.assign";
    
    private int repeatPrecompute;
    private int repeatPreparate;
    
    private Collection<StandardEntity> entities;
    
    //    private List<StandardEntity> centerList;
//    private List<EntityID> centerIDs;
    private Map<Integer, List<StandardEntity>> clusterEntitiesList;
    private List<List<EntityID>> clusterEntityIDsList;
    
    private int clusterSize;
    
    private boolean assignAgentsFlag;
    
    private Map<EntityID, Set<EntityID>> shortestPathGraph;
    
    //GMM param
    private NormalDistribution nd;
    
    private List<StandardEntity> initList;
    private double[] pis;
    private Point2D[] mus;
    private double[][][] sigs;
    private double[][] gammas;
    private double[] gammaSums;
    
    
    public RioneGMM(AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager, DevelopData developData) {
        super(ai, wi, si, moduleManager, developData);
        this.repeatPrecompute = developData.getInteger("sample.module.SampleKMeans.repeatPrecompute", 7);
        this.repeatPreparate = developData.getInteger("sample.module.SampleKMeans.repeatPreparate", 30);
        this.clusterSize = developData.getInteger("sample.module.SampleKMeans.clusterSize", 10);
        this.assignAgentsFlag = developData.getBoolean("sample.module.SampleKMeans.assignAgentsFlag", true);
        this.clusterEntityIDsList = new ArrayList<>();
//        this.centerIDs = new ArrayList<>();
        this.clusterEntitiesList = new HashMap<>();
//        this.centerList = new ArrayList<>();
        this.entities = wi.getEntitiesOfType(
                StandardEntityURN.ROAD,
                StandardEntityURN.HYDRANT,
                StandardEntityURN.BUILDING,
                StandardEntityURN.REFUGE,
                StandardEntityURN.GAS_STATION,
                StandardEntityURN.AMBULANCE_CENTRE,
                StandardEntityURN.FIRE_STATION,
                StandardEntityURN.POLICE_OFFICE
        );
        this.initList = new ArrayList<>();
        this.pis = new double[this.clusterSize];
        this.mus = new Point2D[this.clusterSize];
        this.sigs = new double[this.clusterSize][2][2];
        this.gammas = new double[this.entities.size()][this.clusterSize];
        this.gammaSums = new double[this.clusterSize];
        
    }
    
    @Override
    public Clustering updateInfo(MessageManager messageManager) {
        super.updateInfo(messageManager);
        if(this.getCountUpdateInfo() >= 2) {
            return this;
        }
//        this.centerList.clear();
        this.initList.clear();
        this.clusterEntitiesList.clear();
        return this;
    }
    
    @Override
    public Clustering precompute(PrecomputeData precomputeData) {
        super.precompute(precomputeData);
        if(this.getCountPrecompute() >= 2) {
            return this;
        }
        this.calcPathBased(this.repeatPrecompute);
        this.entities = null;
        // write
        precomputeData.setInteger(KEY_CLUSTER_SIZE, this.clusterSize);
//        precomputeData.setEntityIDList(KEY_CLUSTER_CENTER, this.centerIDs);
        for(int i = 0; i < this.clusterSize; i++) {
            precomputeData.setEntityIDList(KEY_CLUSTER_ENTITY + i, this.clusterEntityIDsList.get(i));
        }
        precomputeData.setBoolean(KEY_ASSIGN_AGENT, this.assignAgentsFlag);
        return this;
    }
    
    @Override
    public Clustering resume(PrecomputeData precomputeData) {
        super.resume(precomputeData);
        if(this.getCountResume() >= 2) {
            return this;
        }
        this.entities = null;
        // read
        this.clusterSize = precomputeData.getInteger(KEY_CLUSTER_SIZE);
//        this.centerIDs = new ArrayList<>(precomputeData.getEntityIDList(KEY_CLUSTER_CENTER));
        this.clusterEntityIDsList = new ArrayList<>(this.clusterSize);
        for(int i = 0; i < this.clusterSize; i++) {
            this.clusterEntityIDsList.add(i, precomputeData.getEntityIDList(KEY_CLUSTER_ENTITY + i));
        }
        this.assignAgentsFlag = precomputeData.getBoolean(KEY_ASSIGN_AGENT);
        return this;
    }
    
    @Override
    public Clustering preparate() {
        super.preparate();
        if(this.getCountPreparate() >= 2) {
            return this;
        }
        this.calcStandard(this.repeatPreparate);
        this.entities = null;
        return this;
    }
    
    @Override
    public int getClusterNumber() {
        //The number of clusters
        return this.clusterSize;
    }
    
    @Override
    public int getClusterIndex(StandardEntity entity){
        return this.getClusterIndex(entity.getID());
    }
    
    @Override
    public int getClusterIndex(EntityID id) {
        for(int i = 0; i < this.clusterSize; i++) {
            if(this.clusterEntityIDsList.get(i).contains(id)) {
                return i;
            }
        }
        return -1;
    }
    
    @Override
    public Collection<StandardEntity> getClusterEntities(int index) {
        List<StandardEntity> result = this.clusterEntitiesList.get(index);
        if(result == null || result.isEmpty()) {
            List<EntityID> list = this.clusterEntityIDsList.get(index);
            result = new ArrayList<>(list.size());
            for(int i = 0; i < list.size(); i++) {
                result.add(i, this.worldInfo.getEntity(list.get(i)));
            }
            this.clusterEntitiesList.put(index, result);
        }
        return result;
    }
    
    @Override
    public Collection<EntityID> getClusterEntityIDs(int index){
        return this.clusterEntityIDsList.get(index);
    }
    
    @Override
    public Clustering calc() {
        return this;
    }
    
    private void calcStandard(int repeat) {
        this.initShortestPath(this.worldInfo);
        
        List<StandardEntity> entityList = new ArrayList<>(this.entities);
        this.clusterEntitiesList = new HashMap<>(this.clusterSize);
        
        this.pis = new double[this.clusterSize];
        this.mus = new Point2D[this.clusterSize];
        this.sigs = new double[this.clusterSize][2][2];
        this.gammas = new double[this.entities.size()][this.clusterSize];
        this.gammaSums = new double[this.entities.size()];
        
        //init list
        for (int index = 0; index < this.clusterSize; index++) {
            this.clusterEntitiesList.put(index, new ArrayList<>());
            this.pis[index] = 0;
            this.mus[index] = new Point2D(0, 0);
            this.sigs[index][0][0] = 0.1;
            this.sigs[index][0][1] = 0;
            this.sigs[index][1][0] = 0;
            this.sigs[index][1][1] = 0.1;
        }
        System.out.println("[" + this.getClass().getSimpleName() + "] Cluster : " + this.clusterSize);
        for (int indexE = 0; indexE < this.entities.size(); indexE++){
            gammaSums[indexE] = 0;
        }
        
        //init parameters
        Random random = new Random();
        double piSum = 0;
        for (int index = 0; index < this.clusterSize; index++){
            StandardEntity initEntity;
            do {
                initEntity = entityList.get(Math.abs(random.nextInt()) % entityList.size());
            } while (this.initList.contains(initEntity));
            
            if(index == this.clusterSize - 1){
                this.pis[index] = 1 - piSum;
            }else{
                this.pis[index] = 1/(double)this.clusterSize;
                piSum += this.pis[index];
            }
            this.mus[index] = getPoint2D(worldInfo.getLocation(initEntity));
            this.sigs[index][0][0] = 0.1;
            this.sigs[index][0][1] = 0;
            this.sigs[index][1][0] = 0;
            this.sigs[index][1][1] = 0.1;
        }
        
        
        //calc Gaussian
        for (int i = 0; i < repeat; i++) {
            for (int index = 0; index < this.clusterSize; index++) {
                double[][] sig = sigs[index];
                gammaSums[index] = 0;
                nd = new NormalDistribution(mus[index], sig);
                for (int indexE = 0; indexE < this.entities.size(); indexE++){
                    gammas[indexE][index] = pis[index] * nd.getProb(getPoint2D(this.worldInfo.getLocation(entityList.get(indexE))));
                    gammaSums[index] += gammas[indexE][index];
                }
            }
            
            for (int index = 0; index < this.clusterSize; index++) {
                Point2D mus2 = new Point2D(0, 0);
                double[][] sigs2 = new double[][]{{0, 0}, {0,0}};
                double Nk = 0;
                
                for (int indexE = 0; indexE < this.entities.size(); indexE++){
                    Point2D x = getPoint2D(this.worldInfo.getLocation(entityList.get(indexE)));
                    double gamma = gammas[indexE][index]/gammaSums[index];
                    
                    mus2 = mus2.translate(gamma*x.getX(), gamma*x.getY());
                    
                    double d1 = x.getX() - mus[index].getX();
                    double d2 = x.getY() - mus[index].getY();
                    sigs2[0][0] += gamma*d1*d1;
                    sigs2[0][1] += gamma*d1*d2;
                    sigs2[1][0] += gamma*d2*d1;
                    sigs2[1][1] += gamma*d2*d2;
                    
                    Nk += gamma;
                }
                
                mus[index] = new Point2D(mus2.getX()/Nk, mus2.getY()/Nk);
                sigs[index] = new double[][]{{sigs2[0][0]/Nk, sigs2[0][1]/Nk},
                        {sigs2[1][0]/Nk, sigs2[1][1]/Nk}};
                pis[index] = Nk/this.entities.size();
                
            }
            
            
            if  (scenarioInfo.isDebugMode()) { System.out.print("*"); }
        }
        
        if  (scenarioInfo.isDebugMode()) { System.out.println(); }
        
        //set entity
        this.clusterEntitiesList.clear();
        for (int index = 0; index < this.clusterSize; index++) {
            this.clusterEntitiesList.put(index, new ArrayList<>());
        }
        for (StandardEntity entity : entityList) {
            this.clusterEntitiesList.get(getMostIndex(worldInfo, entity)).add(entity);
        }
        
        if(this.assignAgentsFlag) {
            List<StandardEntity> firebrigadeList = new ArrayList<>(this.worldInfo.getEntitiesOfType(StandardEntityURN.FIRE_BRIGADE));
            List<StandardEntity> policeforceList = new ArrayList<>(this.worldInfo.getEntitiesOfType(StandardEntityURN.POLICE_FORCE));
            List<StandardEntity> ambulanceteamList = new ArrayList<>(this.worldInfo.getEntitiesOfType(StandardEntityURN.AMBULANCE_TEAM));
            
            this.assignAgents(this.worldInfo, firebrigadeList);
            this.assignAgents(this.worldInfo, policeforceList);
            this.assignAgents(this.worldInfo, ambulanceteamList);
        }
        
        for (int index = 0; index < this.clusterSize; index++) {
            List<StandardEntity> entities = this.clusterEntitiesList.get(index);
            List<EntityID> list = new ArrayList<>(entities.size());
            for(int i = 0; i < entities.size(); i++) {
                list.add(i, entities.get(i).getID());
            }
            this.clusterEntityIDsList.add(index, list);
        }
    }
    
    private void calcPathBased(int repeat) {
        List<StandardEntity> entityList = new ArrayList<>(this.entities);
        this.clusterEntitiesList = new HashMap<>(this.clusterSize);
        
        this.pis = new double[this.clusterSize];
        this.mus = new Point2D[this.clusterSize];
        this.sigs = new double[this.clusterSize][2][2];
        this.gammas = new double[this.entities.size()][this.clusterSize];
        this.gammaSums = new double[this.clusterSize];
        
        //init list
        for (int index = 0; index < this.clusterSize; index++) {
            this.clusterEntitiesList.put(index, new ArrayList<>());
            this.pis[index] = 0;
            this.mus[index] = new Point2D(0, 0);
            this.sigs[index][0][0] = 0.1;
            this.sigs[index][0][1] = 0;
            this.sigs[index][1][0] = 0;
            this.sigs[index][1][1] = 0.1;
        }
        for (int indexE = 0; indexE < this.entities.size(); indexE++){
            gammaSums[indexE] = 0;
        }
        
        //init parameters
        Random random = new Random();
        double piSum = 0;
        for (int index = 0; index < this.clusterSize; index++){
            StandardEntity initEntity;
            do {
                initEntity = entityList.get(Math.abs(random.nextInt()) % entityList.size());
            } while (this.initList.contains(initEntity));
            
            if(index == this.clusterSize - 1){
                this.pis[index] = 1 - piSum;
            }else{
                this.pis[index] = 1/(double)this.clusterSize;
                piSum += this.pis[index];
            }
            this.mus[index] = getPoint2D(worldInfo.getLocation(initEntity));
            this.sigs[index][0][0] = 0.1;
            this.sigs[index][0][1] = 0;
            this.sigs[index][1][0] = 0;
            this.sigs[index][1][1] = 0.1;
        }
        
        
        //calc Gaussian
        for (int i = 0; i < repeat; i++) {
            for (int index = 0; index < this.clusterSize; index++) {
                double[][] sig = sigs[index];
                nd = new NormalDistribution(mus[index], sig);
                gammaSums[index] = 0;
                
                for (int indexE = 0; indexE < this.entities.size(); indexE++){
                    gammas[indexE][index] = pis[index] * nd.getProb(getPoint2D(this.worldInfo.getLocation(entityList.get(indexE))));
                    gammaSums[index] += gammas[indexE][index];
                }
            }
            
            for (int index = 0; index < this.clusterSize; index++) {
                Point2D mus2 = new Point2D(0, 0);
                double[][] sigs2 = new double[][]{{0, 0}, {0,0}};
                double Nk = 0;
                
                for (int indexE = 0; indexE < this.entities.size(); indexE++){
                    Point2D x = getPoint2D(this.worldInfo.getLocation(entityList.get(indexE)));
                    double gamma = gammas[indexE][index]/gammaSums[index];
                    
                    mus2 = mus2.translate(gamma*x.getX(), gamma*x.getY());
                    
                    double d1 = x.getX() - mus[index].getX();
                    double d2 = x.getY() - mus[index].getY();
                    sigs2[0][0] += gamma*d1*d1;
                    sigs2[0][1] += gamma*d1*d2;
                    sigs2[1][0] += gamma*d2*d1;
                    sigs2[1][1] += gamma*d2*d2;
                    
                    Nk += gamma;
                }
                
                mus[index] = new Point2D(mus2.getX()/Nk, mus2.getY()/Nk);
                sigs[index] = new double[][]{{sigs2[0][0]/Nk, sigs2[0][1]/Nk},
                        {sigs2[1][0]/Nk, sigs2[1][1]/Nk}};
                pis[index] = Nk/this.entities.size();
            }
            
            
            if  (scenarioInfo.isDebugMode()) { System.out.print("*"); }
        }
        
        if  (scenarioInfo.isDebugMode()) { System.out.println(); }
        
        //set entity
        this.clusterEntitiesList.clear();
        for (int index = 0; index < this.clusterSize; index++) {
            this.clusterEntitiesList.put(index, new ArrayList<>());
        }
        for (StandardEntity entity : entityList) {
            this.clusterEntitiesList.get(getMostIndex(worldInfo, entity)).add(entity);
        }
        
        if(this.assignAgentsFlag) {
            List<StandardEntity> firebrigadeList = new ArrayList<>(this.worldInfo.getEntitiesOfType(StandardEntityURN.FIRE_BRIGADE));
            List<StandardEntity> policeforceList = new ArrayList<>(this.worldInfo.getEntitiesOfType(StandardEntityURN.POLICE_FORCE));
            List<StandardEntity> ambulanceteamList = new ArrayList<>(this.worldInfo.getEntitiesOfType(StandardEntityURN.AMBULANCE_TEAM));
            
            this.assignAgents(this.worldInfo, firebrigadeList);
            this.assignAgents(this.worldInfo, policeforceList);
            this.assignAgents(this.worldInfo, ambulanceteamList);
        }
        
        for (int index = 0; index < this.clusterSize; index++) {
            List<StandardEntity> entities = this.clusterEntitiesList.get(index);
            List<EntityID> list = new ArrayList<>(entities.size());
            for(int i = 0; i < entities.size(); i++) {
                list.add(i, entities.get(i).getID());
            }
            this.clusterEntityIDsList.add(index, list);
        }
    }
    
    
    private void assignAgents(WorldInfo world, List<StandardEntity> agentList) {
        int clusterIndex = 0;
        while (agentList.size() > 0) {
            StandardEntity agent = this.getMostAgent(world, agentList, clusterIndex);
            this.clusterEntitiesList.get(clusterIndex).add(agent);
            agentList.remove(agent);
            clusterIndex++;
            if (clusterIndex >= this.clusterSize) {
                clusterIndex = 0;
            }
        }
    }
    
    
    private int getMostIndex(WorldInfo world, StandardEntity entity) {
        int result = 0;
        double min = Double.MAX_VALUE;
        for(int index = 0; index < this.clusterSize; index++) {
            double[][] sig = sigs[index];
            nd = new NormalDistribution(mus[index], sig);
            double prob = pis[index]*nd.getProb(getPoint2D(world.getLocation(entity)));
            if(prob < min){
                result = index;
                min = prob;
            }
        }
        return result;
    }
    
    private  StandardEntity getMostAgent(WorldInfo worldInfo, List<StandardEntity> srcAgentList,int clasterIndex){
        StandardEntity result = null;
        for (StandardEntity agent : srcAgentList) {
            Human human = (Human)agent;
            if (result == null) {
                result = agent;
            }
            else {
                if (this.compareProb(worldInfo, clasterIndex, result, worldInfo.getPosition(human)).equals(worldInfo.getPosition(human))) {
                    result = agent;
                }
            }
        }
        return result;
    }
    
    
    private StandardEntity getNearEntity(WorldInfo worldInfo, List<StandardEntity> srcEntityList, int targetX, int targetY) {
        StandardEntity result = null;
        for (StandardEntity entity : srcEntityList) {
            result = (result != null) ? this.compareLineDistance(worldInfo, targetX, targetY, result, entity) : entity;
        }
        return result;
    }
    
    private Point2D getEdgePoint(Edge edge) {
        Point2D start = edge.getStart();
        Point2D end = edge.getEnd();
        return new Point2D(((start.getX() + end.getX()) / 2.0D), ((start.getY() + end.getY()) / 2.0D));
    }
    
    private Point2D getPoint2D(Pair<Integer, Integer> pair){
        if(pair == null){
            return new Point2D(0, 0);
        }else if(pair.first() == null || pair.second() == null){
            return new Point2D(0, 0);
        }
        return new Point2D(pair.first(), pair.second());
    }
    
    
    private double getDistance(double fromX, double fromY, double toX, double toY) {
        double dx = fromX - toX;
        double dy = fromY - toY;
        return Math.hypot(dx, dy);
    }
    
    private double getDistance(Pair<Integer, Integer> from, Point2D to) {
        return getDistance(from.first(), from.second(), to.getX(), to.getY());
    }
    
    private double getDistance(Pair<Integer, Integer> from, Edge to) {
        return getDistance(from, getEdgePoint(to));
    }
    
    private double getDistance(Pair<Integer, Integer> from, Pair<Integer, Integer> to) {
        return getDistance(from.first(), from.second(), to.first(), to.second());
    }
    
    private double getDistance(Point2D from, Point2D to) {
        return getDistance(from.getX(), from.getY(), to.getX(), to.getY());
    }
    
    private double getDistance(Edge from, Edge to) {
        return getDistance(getEdgePoint(from), getEdgePoint(to));
    }
    
    private StandardEntity compareLineDistance(WorldInfo worldInfo, int targetX, int targetY, StandardEntity first, StandardEntity second) {
        Pair<Integer, Integer> firstLocation = worldInfo.getLocation(first);
        Pair<Integer, Integer> secondLocation = worldInfo.getLocation(second);
        double firstDistance = getDistance(firstLocation.first(), firstLocation.second(), targetX, targetY);
        double secondDistance = getDistance(secondLocation.first(), secondLocation.second(), targetX, targetY);
        return (firstDistance < secondDistance ? first : second);
    }
    
    private StandardEntity getNearEntity(WorldInfo worldInfo, List<StandardEntity> srcEntityList, StandardEntity targetEntity) {
        StandardEntity result = null;
        for (StandardEntity entity : srcEntityList) {
            result = (result != null) ? this.comparePathDistance(worldInfo, targetEntity, result, entity) : entity;
        }
        return result;
    }
    
    private StandardEntity comparePathDistance(WorldInfo worldInfo, StandardEntity target, StandardEntity first, StandardEntity second) {
        double firstDistance = getPathDistance(worldInfo, shortestPath(target.getID(), first.getID()));
        double secondDistance = getPathDistance(worldInfo, shortestPath(target.getID(), second.getID()));
        return (firstDistance < secondDistance ? first : second);
    }
    
    private StandardEntity compareProb(WorldInfo worldInfo, int index, StandardEntity first, StandardEntity second) {
        double[][] sig = sigs[index];
        nd = new NormalDistribution(mus[index], sig);
        double firstProb = nd.getProb(getPoint2D(worldInfo.getLocation(first)));
        double secondProb = nd.getProb(getPoint2D(worldInfo.getLocation(second)));
        return (firstProb < secondProb ? first : second);
    }
    
    private double getPathDistance(WorldInfo worldInfo, List<EntityID> path) {
        if (path == null) return Double.MAX_VALUE;
        if (path.size() <= 1) return 0.0D;
        
        double distance = 0.0D;
        int limit = path.size() - 1;
        
        Area area = (Area)worldInfo.getEntity(path.get(0));
        distance += getDistance(worldInfo.getLocation(area), area.getEdgeTo(path.get(1)));
        area = (Area)worldInfo.getEntity(path.get(limit));
        distance += getDistance(worldInfo.getLocation(area), area.getEdgeTo(path.get(limit - 1)));
        
        for(int i = 1; i < limit; i++) {
            area = (Area)worldInfo.getEntity(path.get(i));
            distance += getDistance(area.getEdgeTo(path.get(i - 1)), area.getEdgeTo(path.get(i + 1)));
        }
        return distance;
    }
    
    private void initShortestPath(WorldInfo worldInfo) {
        Map<EntityID, Set<EntityID>> neighbours = new LazyMap<EntityID, Set<EntityID>>() {
            @Override
            public Set<EntityID> createValue() {
                return new HashSet<>();
            }
        };
        for (Entity next : worldInfo) {
            if (next instanceof Area) {
                Collection<EntityID> areaNeighbours = ((Area) next).getNeighbours();
                neighbours.get(next.getID()).addAll(areaNeighbours);
            }
        }
        for (Map.Entry<EntityID, Set<EntityID>> graph : neighbours.entrySet()) {// fix graph
            for (EntityID entityID : graph.getValue()) {
                neighbours.get(entityID).add(graph.getKey());
            }
        }
        this.shortestPathGraph = neighbours;
    }
    
    private List<EntityID> shortestPath(EntityID start, EntityID... goals) {
        return shortestPath(start, Arrays.asList(goals));
    }
    
    private List<EntityID> shortestPath(EntityID start, Collection<EntityID> goals) {
        List<EntityID> open = new LinkedList<>();
        Map<EntityID, EntityID> ancestors = new HashMap<>();
        open.add(start);
        EntityID next;
        boolean found = false;
        ancestors.put(start, start);
        do {
            next = open.remove(0);
            if (isGoal(next, goals)) {
                found = true;
                break;
            }
            Collection<EntityID> neighbours = shortestPathGraph.get(next);
            if (neighbours.isEmpty()) continue;
            
            for (EntityID neighbour : neighbours) {
                if (isGoal(neighbour, goals)) {
                    ancestors.put(neighbour, next);
                    next = neighbour;
                    found = true;
                    break;
                }
                else if (!ancestors.containsKey(neighbour)) {
                    open.add(neighbour);
                    ancestors.put(neighbour, next);
                }
            }
        } while (!found && !open.isEmpty());
        if (!found) {
            // No path
            return null;
        }
        // Walk back from goal to start
        EntityID current = next;
        List<EntityID> path = new LinkedList<>();
        do {
            path.add(0, current);
            current = ancestors.get(current);
            if (current == null) throw new RuntimeException("Found a node with no ancestor! Something is broken.");
        } while (current != start);
        return path;
    }
    
    private boolean isGoal(EntityID e, Collection<EntityID> test) {
        return test.contains(e);
    }
    
    private class NormalDistribution{
        private double[][] vcm;
        private double[][] invVcm;
        private Point2D means;
        
        NormalDistribution(Point2D means, double[][] vcm){
            setVcm(vcm);
            setInvVcm(getInv(vcm));
            setMeans(means);
        }
        
        NormalDistribution(){
            this(null, null);
        }
        
        public void setVcm(double[][] vcm){
            this.vcm = vcm;
        }
        
        public void setInvVcm(double[][] invVcm){
            this.invVcm = invVcm;
        }
        
        public void setMeans(Point2D means){
            this.means = means;
        }
        
        private double getProb(Point2D x){
            if(x == null){
                return Double.NaN;
            }
            
            double a = Math.exp(dot2DTx2D(dot2DTx2M(prod2D(-0.5, subPoint2D(x, this.means)), this.invVcm), subPoint2D(x, this.means)));
            double b = Math.PI*2 * Math.sqrt(getDet(this.vcm));
            if(a < 0.001) a = 0.001;
            return a / b;
        }
        
        public double[][] getInv(double[][] m){
            double a = m[0][0];
            double b = m[0][1];
            double c = m[1][0];
            double d = m[1][1];
            
            double s = a*d - b*c;
            if(s == 0){
                return null;
            }
            
            return new double[][]{{d/s, -b/s}, {-c/s, a/s}};
        }
        
        public double getDet(double[][] m){
            double a = m[0][0];
            double b = m[0][1];
            double c = m[1][0];
            double d = m[1][1];
            
            return a*d - b*c;
        }
        
        public double[] subPoint2D(Point2D a, Point2D b){
            return new double[]{a.getX() - b.getY(), a.getY() - b.getY()};
        }
        
        public double[] prod2D(double k, double[] m){
            double a = m[0];
            double b = m[1];
            
            return new double[]{k*a, k*b};
        }
        
        public double dot2DTx2D(double[] a, double[] b){
            return a[0]*b[0] + a[1]*b[1];
        }
        
        public double[] dot2DTx2M(double[] a, double[][] b){
            return new double[]{a[0]*b[0][0] + a[1]*b[1][0], a[0]*b[0][1] + a[1]*b[1][1]};
        }
    }
}
