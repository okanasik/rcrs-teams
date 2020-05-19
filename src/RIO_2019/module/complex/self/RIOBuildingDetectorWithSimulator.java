package RIO_2019.module.complex.self;

import adf.agent.action.fire.ActionExtinguish;
import adf.agent.communication.MessageManager;
import adf.agent.communication.standard.bundle.MessageUtil;
import adf.agent.communication.standard.bundle.centralized.CommandAmbulance;
import adf.agent.communication.standard.bundle.centralized.CommandFire;
import adf.agent.communication.standard.bundle.information.*;
import adf.agent.develop.DevelopData;
import adf.agent.info.AgentInfo;
import adf.agent.info.ScenarioInfo;
import adf.agent.info.WorldInfo;
import adf.agent.module.ModuleManager;
import adf.agent.precompute.PrecomputeData;
import adf.component.communication.CommunicationMessage;
import adf.component.module.algorithm.Clustering;
import adf.component.module.complex.BuildingDetector;
import rescuecore2.config.Config;
import rescuecore2.misc.geometry.Vector2D;
import rescuecore2.standard.entities.*;
import rescuecore2.standard.entities.StandardEntityConstants.Fieryness;
import rescuecore2.standard.kernel.comms.ChannelCommunicationModel;
import rescuecore2.worldmodel.EntityID;

import java.awt.*;
import java.util.*;
import java.util.List;


import static rescuecore2.standard.entities.StandardEntityURN.*;

public class RIOBuildingDetectorWithSimulator extends BuildingDetector
{
    private EntityID result;
    
    private Clustering clustering;

    private Simulator simulator;

    private Collection<EntityID> agentPositions;
    private Map<EntityID, Integer> sentTimeMap;
    private int sendingAvoidTimeReceived;
    private int sendingAvoidTimeSent;


    private int maxExtinguishPower;

    // flag & communication
    private boolean isRadio = true;
    private int channelMax = 0;
    int voice = 256;
    int voiceCount = 1;
    private int bandwidth = 1024;
    private int devidedBandwidth;
    // Tactics~ で使われている帯域量 は 30Byte とする
    private final int USED_BANDWIDTH = 30;
    
    
    public RIOBuildingDetectorWithSimulator(AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager, DevelopData developData)
    {
        super(ai, wi, si, moduleManager, developData);
        switch (si.getMode())
        {
            case PRECOMPUTATION_PHASE:
                this.clustering = moduleManager.getModule("RIOBuildingDetectorWithSimulator.Clustering", "RIO_2019.module.algorithm.RioneKmeansPP");
                break;
            case PRECOMPUTED:
                this.clustering = moduleManager.getModule("RIOBuildingDetectorWithSimulator.Clustering", "RIO_2019.module.algorithm.RioneKmeansPP");
                break;
            case NON_PRECOMPUTE:
                this.clustering = moduleManager.getModule("RIOBuildingDetectorWithSimulator.Clustering", "RIO_2019.module.algorithm.RioneKmeansPP");
                break;
        }
        registerModule(this.clustering);
        
        this.agentPositions = new HashSet<>();
        this.sentTimeMap = new HashMap<>();
        this.simulator = new Simulator();
        this.maxExtinguishPower = scenarioInfo.getFireExtinguishMaxSum();
        //radio
        this.channelMax = this.scenarioInfo.getCommsChannelsCount();
        if(channelMax < 2) isRadio = false; // 最大チャンネル数が2以下で通信不可
        Config config = this.scenarioInfo.getRawConfig();
        bandwidth = config.getIntValue(ChannelCommunicationModel.PREFIX+1+".bandwidth");
        voice = config.getIntValue(ChannelCommunicationModel.PREFIX+0+".messages.size");
        voiceCount = config.getIntValue(ChannelCommunicationModel.PREFIX+0+".messages.max");
        int numAgents = this.worldInfo.getEntitiesOfType(AMBULANCE_TEAM,FIRE_BRIGADE,POLICE_FORCE).size();
        int numCenter = this.worldInfo.getEntitiesOfType(StandardEntityURN.AMBULANCE_CENTRE, StandardEntityURN.FIRE_STATION, StandardEntityURN.POLICE_OFFICE).size();
        this.devidedBandwidth = bandwidth / (numAgents + numCenter);
    }
    
    
    @Override
    public BuildingDetector updateInfo(MessageManager messageManager)
    {
    	/*if(this.result != null) {
            Building building = (Building)this.worldInfo.getEntity(this.result);
            if(building.getFieryness() >= 1) {
            	System.out.println(building.getFieryness()+" ID:"+ agentInfo.me().getID().getValue());

                    CommandFire message = new CommandFire(
                            true,
                            agentInfo.me().getID(),
                            building.getID(),
                            CommandFire.ACTION_AUTONOMY
                    );
                    messageManager.addMessage(message);

        }
            }*/

        simulator.step();

        // 帯域の制限
        if(channelMax >= 2) isRadio = true;

        // 視界情報の更新
        Set<EntityID> changedEntities = this.worldInfo.getChanged().getChangedEntities();
        changedEntities.add(this.agentInfo.me().getID());
        
        StandardEntity myEntity = this.agentInfo.me();
        StandardEntity nearAT = null;
        StandardEntity nearPF = null;
        List<Building> burningBuildings = new ArrayList<Building>();
        
        // 都度for文を回さないように,changedEntitiesの参照を極力まとめる
        for(EntityID id : changedEntities){
            StandardEntity entity = this.worldInfo.getEntity(id);
            
            // 視界内のATを一人取得(一番最後のAT)
            if(entity instanceof AmbulanceTeam){
                nearAT = entity;
            }
            // 視界内のPFを一人取得(一番最後のPF)
            else if(entity instanceof PoliceForce){
                nearPF = entity;
            }
            // 視界内の燃えている建物を取得
            else if(entity instanceof Building && ((Building) entity).isOnFire()){
                burningBuildings.add((Building)entity);
            }
        }
        
        // ATに救助命令を送信（できているか不明）
        if(nearAT != null && ((Human)myEntity).getBuriedness() > 0){
            messageManager.addMessage(new CommandAmbulance(isRadio, nearAT.getID(), myEntity.getID(), MessageAmbulanceTeam.ACTION_RESCUE));
        }
        
        // FBに消火命令を送信（できているか不明）
        if(burningBuildings.size() > 0){
            messageManager.addMessage(new CommandFire(isRadio, null, burningBuildings.get(0).getID(), CommandFire.ACTION_EXTINGUISH));
        }
        
        FireBrigade agent = (FireBrigade) agentInfo.me();
        for(CommunicationMessage message: messageManager.getReceivedMessageList()){
            if(message instanceof MessageFireBrigade){
                MessageFireBrigade messageFB = (MessageFireBrigade) message;
                if(messageFB.getAction() == 2){ //ActionExtinguish
                    RIOBuilding building = simulator.getRioBuilding(worldInfo.getEntity(messageFB.getTargetID()));
                    int w = (int) ((double) building.getWaterNeeded() * 1.0);
                    w = Math.min(w, this.maxExtinguishPower);
                    w = Math.min(w, agent.getWater() - 1);
                    building.setWaterQuantity(w);
                }
            }else  if(message instanceof MessageBuilding){
                MessageBuilding mb = (MessageBuilding)message;
                if(!changedEntities.contains(mb.getBuildingID())) {
                    MessageUtil.reflectMessage(this.worldInfo, mb);
                }
            }
        }
        
        super.updateInfo(messageManager);
        if (this.getCountUpdateInfo() >= 2)
        {
            return this;
        }
        
        return this;
    }
    
    @Override
    public BuildingDetector calc()
    {
        
        FireBrigade agent = (FireBrigade)this.agentInfo.me();
        
        if(simulator == null){
            System.out.print("#ID:" + agent.getID().getValue() + " Simulator is NULL!!");
        }else{
            for (Building building: worldInfo.getFireBuildings()){
                System.out.println("#ID:" + agent.getID().getValue() +
                        " Building(" + building.getID().getValue() + ")");
                
                if(simulator.getRioBuilding(building)!=null){
                    System.out.println(" EstTemp: " + simulator.getRioBuilding(building).getTemperature() +
                            " TheTemp: " + building.getTemperature());
                }else{
                    System.out.println("RIOBuilding is NULL!!");
                }
                
            }
        }
        
        
        this.result = this.calcInSight();
        if(this.result!=null) {
            return this;
        }
        this.result = this.calcTargetInCluster();
        if (this.result == null)
        {
            this.result = this.calcTargetInWorld();
        }
        return this;
    }
    //nishida
    //視界内の燃えてる建物を優先
    private EntityID calcInSight() {
        Collection<EntityID> en = this.worldInfo.getChanged().getChangedEntities();
        List<StandardEntity> ses = new ArrayList<>();
        for(EntityID entity:en) {
            StandardEntity se = this.worldInfo.getEntity(entity);
            ses.add(se);
        }
        if(!ses.isEmpty()) {
            List<Building> targets = new ArrayList<Building>();
            targets = filterFiery(ses);
            if(targets!=null && !targets.isEmpty()) {
                Collections.sort(targets, new DistanceSorter(worldInfo, agentInfo.me()));
                Building selectedBuilding = targets.get(0);
                return selectedBuilding.getID();
            }
        }
        return null;
    }
    
    private List<Building> filterFiery(Collection<? extends StandardEntity> input) {
        ArrayList<Building> fireBuildings = new ArrayList<>();
        for (StandardEntity entity : input) {
            if (entity instanceof Building && ((Building) entity).isOnFire()) {
                fireBuildings.add((Building) entity);
            }
        }
        if(!fireBuildings.isEmpty())
            return filterFieryness(fireBuildings);
        return null;
    }
    //nishida
    //燃焼度の低いものを優先
    private  List<Building> filterFieryness(Collection<? extends StandardEntity> input){
        ArrayList<Building> fireBuildings = new ArrayList<>();
        if(!input.isEmpty()) {
            for (StandardEntity entity : input) {
                if (entity instanceof Building
                        && ((Building) entity).isOnFire()
                        &&((Building) entity).getFierynessEnum() == Fieryness.HEATING) {
                    fireBuildings.add((Building) entity);
                }
            }
            if(!fireBuildings.isEmpty())
                return fireBuildings;
            
            for (StandardEntity entity : input) {
                if (entity instanceof Building
                        && ((Building) entity).isOnFire()
                        &&((Building) entity).getFierynessEnum() == Fieryness.BURNING) {
                    fireBuildings.add((Building) entity);
                }
            }
            if(!fireBuildings.isEmpty())
                return fireBuildings;
            
            for (StandardEntity entity : input) {
                if (entity instanceof Building
                        && ((Building) entity).isOnFire()
                        &&((Building) entity).getFierynessEnum() == Fieryness.INFERNO) {
                    fireBuildings.add((Building) entity);
                }
            }
            return fireBuildings;
        }
        return null;
    }
    
    private EntityID calcTargetInCluster()
    {
        int clusterIndex = this.clustering.getClusterIndex(this.agentInfo.getID());
        Collection<StandardEntity> elements = this.clustering.getClusterEntities(clusterIndex);
        if (elements == null || elements.isEmpty())
        {
            return null;
        }
        StandardEntity me = this.agentInfo.me();
        List<StandardEntity> agents = new ArrayList<>(this.worldInfo.getEntitiesOfType(StandardEntityURN.FIRE_BRIGADE));
        Set<StandardEntity> fireBuildings = new HashSet<>();
        for (StandardEntity entity : elements)
        {
            if (entity instanceof Building && ((Building) entity).isOnFire())
            {
                fireBuildings.add(entity);
            }
        }
        
        for (StandardEntity entity : fireBuildings)
        {
            if (agents.isEmpty())
            {
                break;
            }
            else if (agents.size() == 1)
            {
                if (agents.get(0).getID().getValue() == me.getID().getValue())
                {
                    return entity.getID();
                }
                break;
            }
            agents.sort(new DistanceSorter(this.worldInfo, entity));
            StandardEntity a0 = agents.get(0);
            StandardEntity a1 = agents.get(1);
            
            if (me.getID().getValue() == a0.getID().getValue() || me.getID().getValue() == a1.getID().getValue())
            {
                return entity.getID();
            }
            else
            {
                agents.remove(a0);
                agents.remove(a1);
            }
        }
        return null;
    }
    
    //nishida
    private EntityID calcTargetInWorld(){
        Collection<StandardEntity> ses = this.worldInfo.getEntitiesOfType(
                StandardEntityURN.BUILDING,
                StandardEntityURN.GAS_STATION,
                StandardEntityURN.AMBULANCE_CENTRE,
                StandardEntityURN.FIRE_STATION,
                StandardEntityURN.POLICE_OFFICE
        );
        List<Building> targets = new ArrayList<Building>();
        targets = filterFiery(ses);
        if(targets!=null && !targets.isEmpty()) {
            Collections.sort(targets, new DistanceSorter(worldInfo, agentInfo.me()));
            Building selectedBuilding = targets.get(0);
            return selectedBuilding.getID();
        }
        return null;
    }
    
    
    @Override
    public EntityID getTarget()
    {
        return this.result;
    }
    
    @Override
    public BuildingDetector precompute(PrecomputeData precomputeData)
    {
        super.precompute(precomputeData);
        System.out.println("precomputed");
        this.simulator.init();
        if (this.getCountPrecompute() >= 2)
        {
            return this;
        }
        return this;
    }
    
    @Override
    public BuildingDetector resume(PrecomputeData precomputeData)
    {
        super.resume(precomputeData);
        System.out.println("resumed");
        //this.simulator.init();
        if (this.getCountPrecompute() >= 2)
        {
            return this;
        }
        return this;
    }
    
    @Override
    public BuildingDetector preparate()
    {
        super.preparate();
        System.out.println("preparated");
        this.simulator.init();
        if (this.getCountPrecompute() >= 2)
        {
            return this;
        }
        return this;
    }
    
    private class DistanceSorter implements Comparator<StandardEntity>
    {
        private StandardEntity reference;
        private WorldInfo worldInfo;
        
        DistanceSorter(WorldInfo wi, StandardEntity reference)
        {
            this.reference = reference;
            this.worldInfo = wi;
        }
        
        public int compare(StandardEntity a, StandardEntity b)
        {
            int d1 = this.worldInfo.getDistance(this.reference, a);
            int d2 = this.worldInfo.getDistance(this.reference, b);
            return d1 - d2;
        }
    }
    private boolean isInBlockade(Human human) {
        if(!human.isXDefined() || !human.isXDefined()) return false;
        int agentX = human.getX();
        int agentY = human.getY();
        StandardEntity positionEntity = this.worldInfo.getPosition(human);
        if(positionEntity instanceof Road){
            Road road = (Road)positionEntity;
            if(road.isBlockadesDefined() && road.getBlockades().size() > 0){
                for(Blockade blockade : worldInfo.getBlockades(road)){
                    if(blockade.getShape().contains(agentX, agentY)){
                        return true;
                    }
                }
            }
        }
        return false;
    }
  
 	
 	/*
 	主に使用できるメソッドは
 	- getEstFireBuildingsで燃えてる建物の集合を取る
 	- getRioBuildingでBuildingをRIOBuildingに変換する
 	  - 変換するといろんなパラメータの推定を取得できる
 	 */
    /**
     * @author AMADA(Ri-one 14th)
     *
     */
    class Simulator{
        public float GAMMA = 0.5f;
        public float AIR_TO_AIR_COEFFICIENT = 0.5f;
        public float AIR_TO_BUILDING_COEFFICIENT = 45f;
        public float WATER_COEFFICIENT = 0.5f;
        public float ENERGY_LOSS = 0.9f;
        public float WIND_DIRECTION = 0.9f;
        public float WIND_RANDOM = 0f;
        public int WIND_SPEED = 0;
        public float RADIATION_COEFFICENT = 1.0f;
        public float TIME_STEP_LENGTH = 1f;
        public float WEIGHT_GRID = 0.2f;
        public float AIR_CELL_HEAT_CAPACITY = 1f;
    
        public final static double INF = 1e50;
        public final static double EPS = 1e-8;
    
        public static final int COLLINEAR = 0;
        public static final int CLOCKWISE = 1;
        public static final int COUNTER_CLOCKWISE = -1;
        public static final int COUN = 0;
        
        public Set monitors;
        public boolean verbose;
        private Simulator me;
    
        ArrayList<Wall> allWalls = new ArrayList<>();
    
        private Rectangle rect=new Rectangle(0,0,0,0);
    
        private int maxX=Integer.MIN_VALUE;
        private int maxY=Integer.MIN_VALUE;
        private int minX=Integer.MAX_VALUE;
        private int minY=Integer.MAX_VALUE;
        private int SAMPLE_SIZE=5000;
        private int CAPACITY;
        private float AIR_CAPACITY=0.2f;
        private float AIR_HEIGHT=30;
        private ArrayList[][] gridToBuilding;
        
        private ArrayList<RIOBuilding> buildings;
        
        private double[][] airTemp;
        
        
        public Simulator(){
            System.out.println("simulator is lunched.");
            me = this;
            monitors = new HashSet();
            verbose = true;
            buildings = new ArrayList<>();
            System.out.println("simulator is initing.");
            //init();
        }
        
        private void init(){
            allWalls.clear();
            for (StandardEntity entity: worldInfo.getEntitiesOfType(StandardEntityURN.BUILDING)){
                if (entity instanceof Building){
                    RIOBuilding rioBuilding = new RIOBuilding(entity);
                    buildings.add(rioBuilding);
                }
            }
            for(RIOBuilding building: buildings){
                building.calcConnectionsAndPaint();
            }
            initializeAir();
            igniteGISFires();
            System.out.println("Simulator was inited.");
        }
    
        private void initializeAir() {
    
            int xSamples=1 + (maxX - minX) / SAMPLE_SIZE;
            int ySamples=1 + (maxY - minY) / SAMPLE_SIZE;
            airTemp=new double[xSamples][ySamples];
            for(int x=0;x<airTemp.length;x++)
                for(int y=0;y<airTemp[x].length;y++)
                    airTemp[x][y]=0;
            CAPACITY=(int)(SAMPLE_SIZE*SAMPLE_SIZE*AIR_HEIGHT*AIR_CAPACITY)/1000000;
            //assign buildings
            gridToBuilding=new ArrayList[xSamples][ySamples];
            for(int x=0;x<gridToBuilding.length;x++)
                for(int y=0;y<gridToBuilding[0].length;y++)
                    gridToBuilding[x][y]=new ArrayList();
            for(RIOBuilding b : buildings) {
                b.findCells();
            }
        }
    
        public void igniteGISFires(){
            for(Iterator it=buildings.iterator();it.hasNext();){
                RIOBuilding b = (RIOBuilding)it.next();
                if(b.getIgnition()!= 0){
                    b.ignite();
                }
            }
        }
        
        public Simulator getSimulator(){
            return me;
        }
        
        public double[][] getAirTemp(){
            return airTemp;
        }
        
        public void setAirTemp(double[][] a){
            airTemp = a;
        }
        
        public void setAirCellTemp(int x, int y, double temp){
            airTemp[x][y] = temp;
        }
        
        public double getAirCellTemp(int x, int y){
            return airTemp[x][y];
        }
    
        public int getMinX(){
            return minX;
        }
    
        public int getMinY(){
            return minY;
        }
    
        public int getMaxX(){
            return maxX;
        }
    
        public int getMaxY(){
            return maxY;
        }
    
        /**
         * BuildingからRIOBuildingへの変換
         * @author AMADA(Ri-one 14th)
         * @param building
         * @return RIObuilding
         */
        RIOBuilding getRioBuilding(Building building){
            for(RIOBuilding b:buildings){
                if(b == null){
                    continue;
                }
                if(b.selfBuilding.equals(building)){
                    return b;
                }
            }
            
            System.out.println("RioBuilding is NOT found!!");
            return null;
        }
        
        /**
         * StandardEntityからRIOBuildingへの変換
         * @author AMADA(Ri-one 14th)
         * @param standardEntity
         * @return RIObuilding
         */
        RIOBuilding getRioBuilding(StandardEntity standardEntity){
            if(standardEntity instanceof Building){
                Building building = (Building)standardEntity;
                return  getRioBuilding(building);
            }
            System.out.println("Entity is NOT Building!!");
            return null;
        }
        
        /**
         * 現在燃えてると思われる建物の集合を返す
         * @author AMADA(Ri-one 14th)
         * @return Collection<RIOBuilding>
         */
        public Collection<RIOBuilding> getEstFireRIOBuildings(){
            ArrayList<RIOBuilding> results = new ArrayList<>();
            for(RIOBuilding building : buildings){
                if(building != null && building.isBurning()){
                    results.add(building);
                }
            }
            return results;
        }
        
        /**
         * 現在燃えてると思われる建物の集合を返す
         * @author AMADA(Ri-one 14th)
         * @return Collection<Building>
         */
        public Collection<Building> getEstFireBuildings(){
            ArrayList<Building> results = new ArrayList<>();
            for(RIOBuilding building : buildings){
                if(building != null && building.isBurning()){
                    results.add(building.getSelfBuilding());
                }
            }
            return results;
        }
        
        //-----------------------Simulator------------------------
        /**
         * core of fire sim
         */
        public void step(){
            burn();
            cool();
            updateGrid();
            exchangeBuilding();
            cool();
        }
        
        private void cool(){
            for (RIOBuilding b: buildings){
                waterCooling(b);
            }
        }
        
        private void burn(){
            for (RIOBuilding b: buildings){
                if (b.getTemperature() >= b.getIgnitionPoint() && b.getFuel()>0 && b.isFlammable()){
                    float consumed = b.getConsum();
                    if (consumed>b.getFuel()){
                        consumed = b.getFuel();
                    }
                    b.setEnergy(b.getEnergy() + consumed);
                    b.setPrevBurned(consumed);
                    b.setFuel(b.getFuel() - consumed);
                    b.setPrevBurned(consumed);
                }else{
                    b.setPrevBurned(0f);
                }
            }
        }
        
        private void waterCooling(RIOBuilding b){
            double lWATER_COEFFICIENT = (b.getFieryness()>0 && b.getFieryness()<4 ? WATER_COEFFICIENT : WATER_COEFFICIENT*GAMMA);
            if (b.getWaterQuantity()>0){
                double dE = b.getTemperature()*b.getCapacity();
                if (dE<=0){
                    return;
                }
                double effect = b.getWaterQuantity()*lWATER_COEFFICIENT;
                int consumed = b.getWaterQuantity();
                if (effect>dE){
                    double pc = 1 - ((effect - dE)/effect);
                    effect *= pc;
                    consumed *= pc;
                }
                b.setWaterQuantity(b.getWaterQuantity() - consumed);
                b.setEnergy(b.getEnergy() - effect);
            }
        }
        
        private void exchangeBuilding(){
            for (Iterator i = buildings.iterator(); i.hasNext(); ){
                RIOBuilding b = (RIOBuilding)i.next();
                exchangeWithAir(b);
            }
            double sumdt = 0;
            Map<RIOBuilding, Double> radiation = new HashMap<RIOBuilding, Double>();
            for (Iterator i = buildings.iterator(); i.hasNext(); ){
                RIOBuilding b = (RIOBuilding)i.next();
                double radEn = b.getRadiationEnergy();
                radiation.put(b, radEn);
            }
            for (Iterator i = buildings.iterator(); i.hasNext(); ){
                RIOBuilding b = (RIOBuilding)i.next();
                double radEn = radiation.get(b);
                List<RIOBuilding> bs = b.getConnectedBuilding();
                List<Float> vs = b.getConnectedValues();
                
                for (int c = 0; c<vs.size(); c++){
                    double oldEnergy = bs.get(c).getEnergy();
                    double connectionValue = vs.get(c);
                    double a = radEn*connectionValue;
                    double sum = oldEnergy + a;
                    bs.get(c).setEnergy(sum);
                }
                b.setEnergy(b.getEnergy() - radEn);
            }
        }
        
        private void exchangeWithAir(RIOBuilding b){
            
            double oldTemperature = b.getTemperature();
            double oldEnergy = b.getEnergy();
            
            if (oldTemperature>100){
                b.setEnergy(oldEnergy - (oldEnergy*0.042));
            }
        }
        
        private void updateGrid(){
            double[][] airtemp = getAirTemp();
            double[][] newairtemp = new double[airtemp.length][airtemp[0].length];
            for (int x = 0; x<airtemp.length; x++){
                for (int y = 0; y<airtemp[0].length; y++){
                    double dt = (averageTemp(x, y) - airtemp[x][y]);
                    double change = (dt*AIR_TO_AIR_COEFFICIENT*TIME_STEP_LENGTH);
                    newairtemp[x][y] = relTemp(airtemp[x][y] + change);
                    if (!(newairtemp[x][y]>-Double.MAX_VALUE && newairtemp[x][y]<Double.MAX_VALUE)){
                        newairtemp[x][y] = Double.MAX_VALUE*0.75;
                    }
                    if (newairtemp[x][y] == Double.NEGATIVE_INFINITY || newairtemp[x][y] == Double.POSITIVE_INFINITY){
                    }
                }
            }
            setAirTemp(newairtemp);
            //setAirTemp(getWindShift().shift(world.getAirTemp(),this));
        }
        
        private double relTemp(double deltaT){
            return Math.max(0, deltaT*ENERGY_LOSS*TIME_STEP_LENGTH);
        }
        
        private double averageTemp(int x, int y){
            //        double rv = (neighbourCellAverage(x,y)+buildingAverage(x,y))/(weightSummBuilding(x,y)+weightSummCells(x,y));
            double rv = neighbourCellAverage(x, y)/weightSummCells(x, y);
            return rv;
        }
        
        private double neighbourCellAverage(int x, int y){
            double total = getTempAt(x + 1, y - 1);
            total += getTempAt(x + 1, y);
            total += getTempAt(x + 1, y + 1);
            total += getTempAt(x, y - 1);
            total += getTempAt(x, y + 1);
            total += getTempAt(x - 1, y - 1);
            total += getTempAt(x - 1, y);
            total += getTempAt(x - 1, y + 1);
            return total*WEIGHT_GRID;
        }
        
        private float weightSummCells(int x, int y){
            return 8*WEIGHT_GRID;
        }
        
        private double getTempAt(int x, int y){
            if (x<0 || y<0 || x >= this.getAirTemp().length || y >= this.getAirTemp()[0].length)
                return 0;
            return this.getAirTemp()[x][y];
        }
    
        //---------------------------util-------------------------
        
        private double getArea(Polygon p) {
            double sum = 0;
            for (int i = 0; i < p.npoints; i++) {
                sum += (
                        ((double) (p.xpoints[i]) * (p.ypoints[(i + 1) % p.npoints])) -
                                ((double) (p.ypoints[i]) * (p.xpoints[(i + 1) % p.npoints]))
                );
            }
            return Math.abs(sum / 2);
        }
    
        private double getPerimeter(Polygon p) {
            double sum = 0;
            for (int i = 0; i < p.npoints; i++) {
                sum += dist(p.xpoints[i], p.ypoints[i], p.xpoints[(i + 1) % p.npoints], p.ypoints[(i + 1) % p.npoints]);
            }
            return sum;
        }
    
        private double dist(double Ax, double Ay, double Bx, double By) {
            return Math.hypot(Ax - Bx, Ay - By);
        }
    
        private int getOrientation(double Ax1, double Ay1, double Ax2, double Ay2, double Bx1, double By1) {
            double v = (Ay2 - Ay1) * (Bx1 - Ax2) - (Ax2 - Ax1) * (By1 - Ay2);
            if (Math.abs(v) < EPS) {
                return COLLINEAR;
            }
            return v > 0 ? CLOCKWISE : COUNTER_CLOCKWISE;
        }
        
        public boolean isAlmostConvex(Polygon p) {
            if(p.npoints <= 3) {
                return true;
            }
            p = getSimplifiedPolygon(p, 0.1);
            if(p.npoints <= 3) {
                return true;
            }
            int ori = COLLINEAR;
            for (int i = 0; i < p.npoints; i++) {
                int ori_ = getOrientation(
                        p.xpoints[i],
                        p.ypoints[i],
                        p.xpoints[(i + 1) % p.npoints],
                        p.ypoints[(i + 1) % p.npoints],
                        p.xpoints[(i + 2) % p.npoints],
                        p.ypoints[(i + 2) % p.npoints]
                );
                if(ori == COLLINEAR) {
                    ori = ori_;
                }
                if(ori_ != ori && ori_ != COLLINEAR) {
                    return false;
                }
            }
            return true;
        }
    
        private Polygon getSimplifiedPolygon(Polygon p, double d) {
            Polygon result = new Polygon();
            int lastX = p.xpoints[0];
            int lastY = p.ypoints[0];
            result.addPoint(lastX, lastY);
            for(int i = 1; i < p.npoints; i++) {
                double v1x = p.xpoints[i] - lastX;
                double v1y = p.ypoints[i] - lastY;
                double v2x = p.xpoints[(i + 1) % p.npoints] - p.xpoints[i];
                double v2y = p.ypoints[(i + 1) % p.npoints] - p.ypoints[i];
                double l1 = Math.hypot(v1x, v1y);
                double l2 = Math.hypot(v2x, v2y);
                if (Math.abs(l1) < EPS || Math.abs(l2) < EPS) {
                    continue;
                }
                v1x /= l1;
                v1y /= l1;
                v2x /= l2;
                v2y /= l2;
                double v3x = v2x - v1x;
                double v3y = v2y - v1y;
                double l3 = Math.hypot(v3x, v3y);
                if (l3 < d) {
                    continue;
                }
                lastX = p.xpoints[i];
                lastY = p.ypoints[i];
                result.addPoint(lastX, lastY);
            }
            return result;
        }
    
        private ArrayList<double[]> getRandomPointsOnSegmentLine(double Ax, double Ay, double Bx, double By, double rate) {
            ArrayList<double[]> result = new ArrayList<>();
            double dx = Bx - Ax;
            double dy = By - Ay;
            double l = Math.hypot(dx, dy);
            double n = (int) (rate * l);
            if(l <= 1e-5) {
                for(int i = 0; i < n; i++) {
                    result.add(new double[]{Ax, By});
                }
                return result;
            }
            dx /= l;
            dy /= l;
            for(int i = 0; i < n; i++) {
                double rand = Math.random();
                result.add(new double[] {Ax + dx * l * rand, Ay + dy * l * rand});
            }
            return result;
        }
    
        /*
        private Rectangle getOffsettedBounds(int off) {
            Rectangle bounds = this.polygon.getBounds();
            Rectangle result = new Rectangle(
                    (int) bounds.getMinX() - off,
                    (int) bounds.getMinY() - off,
                    (int) bounds.getWidth() + 2 * off,
                    (int) bounds.getHeight() + 2 * off
            );
            return result;
        }
        */
    
        private void getRandomUnitVector(double result[]) {
            double r = Math.random() * Math.PI * 2;
            result[0] = Math.cos(r);
            result[1] = Math.sin(r);
        }
    
        public Point intersect(Point a, Point b, Point c, Point d){
            float[] rv=intersect(new float[]{a.x,a.y,b.x,b.y,c.x,c.y,d.x,d.y});
            if(rv==null)return null;
            return new Point((int)rv[0],(int)rv[1]);
        }
    
        public float[] intersect(float[]points){
            float[] l1=getAffineFunction(points[0],points[1],points[2],points[3]);
            float[] l2=getAffineFunction(points[4],points[5],points[6],points[7]);
            float[] crossing;
            if(l1==null&&l2==null){
                return null;
            }
            else if(l1==null&&l2!=null) {
                crossing= intersect(l2[0],l2[1],points[0]);
            }
            else if(l1!=null&&l2==null){
                crossing= intersect(l1[0],l1[1],points[4]);
            }
            else{
                crossing =intersect(l1[0],l1[1],l2[0],l2[1]);
            }
            if (crossing==null){
                return null;
            }
            if(!(inBounds(points[0],points[1],points[2],points[3],crossing[0],crossing[1])&&
                    inBounds(points[4],points[5],points[6],points[7],crossing[0],crossing[1]))) return null;
            return crossing;
        }
    
        public float[] intersect(float m1, float b1, float m2, float b2){
            if(m1==m2){
                return null;
            }
            float x=(b2-b1)/(m1-m2);
            float y=m1*x+b1;
            return new float[]{x,y};
        }
    
        public float[] intersect(float m1, float b1, float x){
            return new float[]{x, m1*x+b1};
        }
    
    
        public float[] getAffineFunction(float x1,float y1,float x2,float y2){
            if(x1==x2)return null;
            float m=(y1-y2)/(x1-x2);
            float b=y1-m*x1;
            return new float[]{m,b};
        }
    
        public boolean inBounds(float bx1,float by1,float bx2, float by2, float x, float y){
            if(bx1<bx2){
                if(x<bx1||x>bx2)return false;
            }else{
                if(x>bx1||x<bx2)return false;
            }
            if(by1<by2){
                if(y<by1||y>by2)return false;
            }else{
                if(y>by1||y<by2)return false;
            }
            return true;
        }
    
        public boolean boundingTest(Polygon p,int x,int y,int w,int h){
            rect.setBounds(x,y,w,h);
            return p.intersects(rect);
        }
    
        public int percent(float x1,float y1, float width, float height,Polygon p){
            int counter=0;
            double dx=width/10;
            double dy=height/10;
            for(int i=0;i<10;i++){
                for(int j=0;j<10;j++){
                    if(p.contains(dx*i+x1,dy*j+y1))counter++;
                }
            }
            return counter;
        }
    }
    
    public class RIOBuilding{
        private Building selfBuilding;
        private List<RIOBuilding> connectedBuilding;
        public Hashtable connectedBuildings = new Hashtable();
        public List<Float>	connectedValues;
        private Hashtable connectedBuildingsTable = new Hashtable();
        private List<EntityID> neighbourIdBuildings;
        private List<EntityID> neighbourFireBuildings;
        private Collection<Wall> walls;
        private short vis_ = 0;
        private double totalWallArea;
        private ArrayList<Wall> allWalls;
        private int ignition=0;
        //private List<Entrance> entrances;
        private double cellCover;
        private double hitRate = 0;
        
        private Polygon polygon = null;
        
        public int[][] cells;
    
        private final static double RADIATION_RAY_RATE = 0.0025;
        
        public RIOBuilding(StandardEntity entity){
            selfBuilding = (Building)entity;
            connectedBuildingsTable = new Hashtable(30);
            neighbourIdBuildings = new ArrayList<>();
            neighbourFireBuildings = new ArrayList<>();
            connectedBuilding = new ArrayList<>();
            this.polygon = (Polygon) (selfBuilding.getShape());
            if (worldInfo.getEntity(agentInfo.getID()) instanceof FireBrigade){
                initWalls();
                initWallValues();
                initSimulatorValues();
            }
        }
        
        public void initWallValues(){
            int totalHits=0;
            int totalRays=0;
            int selfHits=0;
            int strange=0;
            for(Iterator w=walls.iterator();w.hasNext();){
                Wall wall=(Wall)w.next();
                totalHits+=wall.hits;
                selfHits+=wall.selfHits;
                totalRays+=wall.rays;
                strange=wall.strange;
            }
            int c=0;
            connectedBuilding = new ArrayList<>();
            connectedValues = new ArrayList<>();
            float base = totalRays;
            for(Enumeration e=connectedBuildings.keys();e.hasMoreElements();c++){
                RIOBuilding b=(RIOBuilding)e.nextElement();
                Integer value=(Integer)connectedBuildings.get(b);
                connectedBuilding.set(c, b);
                connectedValues.set(c,value.floatValue()/base);
            }
        }
        
        public void initWalls(){
            
            int fx = selfBuilding.getApexList()[0];
            int fy = selfBuilding.getApexList()[1];
            int lx = fx;
            int ly = fy;
            Wall w;
            walls = new ArrayList<Wall>();
            //allWalls = new ArrayList<Wall>();
            
            for (int n = 2; n<selfBuilding.getApexList().length; n++){
                int tx = selfBuilding.getApexList()[n];
                int ty = selfBuilding.getApexList()[++n];
                w = new Wall(lx, ly, tx, ty, this, 0.0025f);
                if (w.validate()){
                    walls.add(w);
                    //simulator.allWalls.add(w);
                    totalWallArea += FLOOR_HEIGHT*1000*w.length;
                }
                lx = tx;
                ly = ty;
            }
            
            w = new Wall(lx, ly, fx, fy, this, 0.0025f);
            walls.add(w);
            //simulator.allWalls.add(w);
            totalWallArea = totalWallArea/1000000d;
        }
    
        public void findCells() {
            LinkedList tmp=new LinkedList();
            for(int x=0;x<simulator.getAirTemp().length;x++)
                for(int y=0;y<simulator.getAirTemp()[0].length;y++){
                    int xv=x*simulator.SAMPLE_SIZE+simulator.getMinX();
                    int yv=y*simulator.SAMPLE_SIZE+simulator.getMinY();
                    if(simulator.boundingTest(this.polygon,xv,yv,simulator.SAMPLE_SIZE,simulator.SAMPLE_SIZE)){
                        int pc=simulator.percent((float)xv,(float)yv,(float)simulator.SAMPLE_SIZE,(float)simulator.SAMPLE_SIZE,this.polygon);
                        if(pc>0){
                            tmp.add(x);
                            tmp.add(y);
                            tmp.add(pc);
                            Object[] o=new Object[]{this,pc};
                            simulator.gridToBuilding[x][y].add(o);
                        }
                    }
                }
            if(tmp.size()>0){
                cells=new int[tmp.size()/3][3];
                Iterator i=tmp.iterator();
                for(int c=0;c<cells.length;c++){
                    cells[c][0]=((Integer)i.next()).intValue();
                    cells[c][1]=((Integer)i.next()).intValue();
                    cells[c][2]=((Integer)i.next()).intValue();
                }
            }
        }
        
        public void setConnectedBuilding(List<RIOBuilding> connectedBuilding){
            this.connectedBuilding = connectedBuilding;
        }
    
        public ArrayList<RIOBuildingConnection> calcConnectionsAndPaint() {
            double maxDist = 20000;
        
            Polygon bp = this.polygon;
            Rectangle bounds = bp.getBounds();
        
            bounds = new Rectangle(
                    (int) (bounds.getMinX() - maxDist),
                    (int) (bounds.getMinY() - maxDist),
                    (int) (bounds.getWidth() + 2 * maxDist),
                    (int) (bounds.getHeight() + 2 * maxDist)
            );
    
    
            Collection<StandardEntity> cands = worldInfo.getObjectsInRectangle(
                    (int) bounds.getMinX(),
                    (int) bounds.getMinY(),
                    (int) bounds.getMaxX(),
                    (int) bounds.getMaxY()
            );
    
            ArrayList<RIOBuilding> aroundBuildings = new ArrayList<>();
            
            for(StandardEntity sent : cands) {
                if(!sent.getStandardURN().equals(StandardEntityURN.BUILDING)) {
                    continue;
                }
                RIOBuilding b = simulator.getRioBuilding(sent);
                b.vis_ = 0;
                aroundBuildings.add(b);
            }
        
            ArrayList<RIOBuildingConnection> result = new ArrayList<>();
        
            int rays = 0;
        
            Polygon selfPolygon = this.polygon;
        
            for(int i = 0; i < selfPolygon.npoints; i++) {
                ArrayList<double[]> randomOrigins = simulator.getRandomPointsOnSegmentLine(
                        selfPolygon.xpoints[i],
                        selfPolygon.ypoints[i],
                        selfPolygon.xpoints[(i + 1) % selfPolygon.npoints],
                        selfPolygon.ypoints[(i + 1) % selfPolygon.npoints],
                        RADIATION_RAY_RATE
                );
            
                rays += randomOrigins.size();
            
                double rv[] = new double[2];
            
                float ray[] = new float[4];
            
                for(double[] o : randomOrigins) {
                
                    simulator.getRandomUnitVector(rv);
                
                    ray[0] = (float) o[0];
                    ray[1] = (float) o[1];
                    ray[2] = (float) (o[0] + rv[0] * maxDist);
                    ray[3] = (float) (o[1] + rv[1] * maxDist);
                
                    RIOBuilding last = null;
                
                    for(RIOBuilding building : aroundBuildings) {
                        Polygon p = this.polygon;
                    
                        for(int j = 0; j < p.npoints; j++) {
                            if(building == this && i == j) {
                                continue;
                            }
                        
                            Point ip = simulator.intersect(
                                    new Point(p.xpoints[j], p.ypoints[j]),
                                    new Point(p.xpoints[(j + 1) % p.npoints], p.ypoints[(j + 1) % p.npoints]),
                                    new Point((int) ray[0], (int) ray[1]),
                                    new Point((int) ray[2], (int) ray[3])
                            );
                            if(ip != null) {
                                ray[2] = (float) ip.getX();
                                ray[3] = (float) ip.getY();
                                last = building;
                            }
                        }
                    }
                
                    if(last != null) {
                        last.vis_++;
                    }
                }
            }
            for(RIOBuilding b : aroundBuildings) {
                if(b.vis_ > 0 && b != this) {
                    result.add(new RIOBuildingConnection(b.getSelfBuilding().getID().getValue(), ((float) b.vis_ / rays) / 1) );
                }
            
            }
            return result;
        }
        
        private int getWaterNeeded() {
            int waterNeeded = 0;
            double currentTemperature = getTemperature();
            int step = 500;
            while (true) {
                currentTemperature = waterCooling(currentTemperature, step);
                waterNeeded += step;
                if (currentTemperature <= 0) {
                    break;
                }
            }
        
            return waterNeeded;
        }
    
        private double waterCooling(double temperature, int water) {
            if (water > 0) {
                double effect = water * 20;
                return (temperature * getCapacity() - effect) / getCapacity();
            }
            return water;
        }
    
        public void setIgnition(int ignition){
            this.ignition=ignition;
        }
        
        public int getIgnition(){
            return ignition;
        }
    
        public void ignite() {
            energy=getCapacity()*getIgnitionPoint()*1.5;
        }
        
        public List<RIOBuilding> getConnectedBuilding(){
            return connectedBuilding;
        }
        
        public void setConnectedValues(List<Float> connectedValues){
            this.connectedValues = connectedValues;
        }
        
        public List<Float> getConnectedValues(){
            return connectedValues;
        }
        
        public Collection<Wall> getWalls(){
            return walls;
        }
        
        public Hashtable getConnectedBuildingsTable(){
            return connectedBuildingsTable;
        }
        
        public ArrayList<Wall> getAllWalls(){
            return allWalls;
        }
        
        public double getHitRate(){
            return hitRate;
        }
        
        public void setHitRate(double hitRate){
            this.hitRate = hitRate;
        }
        
        public void setNeighbourIdBuildings(List<EntityID> neighbourIdBuildings){
            this.neighbourIdBuildings = neighbourIdBuildings;
        }
        
        public void setNeighbourFireBuildings(List<EntityID> neighbourFireBuildings){
            this.neighbourFireBuildings = neighbourFireBuildings;
        }
        
        public List<EntityID> getNeighbourIdBuildings(){
            return neighbourIdBuildings;
        }
        
        public List<EntityID> getNeighbourFireBuildings(){
            return neighbourFireBuildings;
        }
        
        public double getCellCover(){
            return cellCover;
        }
        
        public void setCellCover(double cellCover){
            this.cellCover = cellCover;
        }
        
        public boolean isBurning(){
            return getFieryness()>0 && getFieryness()<4;
        }
        
        ///////////////////////////////////FIRE SIMULATOR PROPERTIES////////////////////////////////////
        static final int FLOOR_HEIGHT = 3;
        float RADIATION_COEFFICIENT = 0.011f;
        static final double STEFAN_BOLTZMANN_CONSTANT = 0.000000056704;
        
        private int startTime = -1;
        private float fuel;
        private float initFuel = -1;
        private float volume;
        private double energy;
        private float prevBurned;
        private float capacity;
        private int waterQuantity;
        private boolean wasEverWatered = false;
        private boolean flammable = true;
        
        public float woodIgnition = 47;
        public float steelIgnition = 47;
        public float concreteIgnition = 47;
        public float woodCapacity = 1.1f;
        public float steelCapacity = 1.0f;
        public float concreteCapacity = 1.5f;
        public float woodEnergy = 2400;
        public float steelEnergy = 800;
        public float concreteEnergy = 350;
        
        public void initSimulatorValues(){
            volume = selfBuilding.getGroundArea()*selfBuilding.getFloors()*FLOOR_HEIGHT;
            fuel = getInitialFuel();
            capacity = (volume*getThermoCapacity());
            energy = 0;
            initFuel = -1;
            prevBurned = 0;
        }
        
        public float getInitialFuel(){
            if (initFuel<0){
                initFuel = (getFuelDensity()*volume);
            }
            return initFuel;
        }
        
        private float getThermoCapacity(){
            switch (selfBuilding.getBuildingCode()){
                case 0:
                    return woodCapacity;
                case 1:
                    return steelCapacity;
                default:
                    return concreteCapacity;
            }
        }
        
        private float getFuelDensity(){
            switch (selfBuilding.getBuildingCode()){
                case 0:
                    return woodEnergy;
                case 1:
                    return steelEnergy;
                default:
                    return concreteEnergy;
            }
        }
        
        public float getIgnitionPoint(){
            switch (selfBuilding.getBuildingCode()){
                case 0:
                    return woodIgnition;
                case 1:
                    return steelIgnition;
                default:
                    return concreteIgnition;
            }
        }
        
        
        public float getConsum(){
            if (fuel == 0){
                return 0;
            }
            float tf = (float)(getTemperature()/1000f);
            float lf = getFuel()/getInitialFuel();
            float f = (float)(tf*lf*0.15);
            if (f<0.005f)
                f = 0.005f;
            return getInitialFuel()*f;
        }
        
        public float getConsume(double bRate){
            if (fuel == 0){
                return 0;
            }
            float tf = (float)(getTemperature()/1000f);
            float lf = fuel/getInitialFuel();
            float f = (float)(tf*lf*bRate);
            if (f<0.005f)
                f = 0.005f;
            return getInitialFuel()*f;
        }
        
        public double getTemperature(){
            double rv = energy/capacity;
            //System.out.println("Energy:" + energy + "/capacity:" + capacity);
            if(Objects.requireNonNull(agentInfo.getChanged()).getChangedEntities().contains(getSelfBuilding().getID())){
                return selfBuilding.isTemperatureDefined() ? selfBuilding.getTemperature() : 0;
            }
            
            if (Double.isNaN(rv)){
                return selfBuilding.isTemperatureDefined() ? selfBuilding.getTemperature() : 0;
            }
            if (rv == Double.NaN || rv == Double.POSITIVE_INFINITY || rv == Double.NEGATIVE_INFINITY)
                rv = Double.MAX_VALUE*0.75;
            return rv;
        }
        
        public int getFieryness(){
            if (!isFlammable())
                return 0;
            if (getTemperature() >= getIgnitionPoint()){
                if (fuel >= getInitialFuel()*0.66)
                    return 1;   // burning, slightly damaged
                if (fuel >= getInitialFuel()*0.33)
                    return 2;   // burning, more damaged
                if (fuel>0)
                    return 3;    // burning, severly damaged
            }
            if (fuel == getInitialFuel())
                if (wasEverWatered)
                    return 4;   // not burnt, but watered-damaged
                else
                    return 0;   // not burnt, no water damage
            if (fuel >= getInitialFuel()*0.66)
                return 5;        // extinguished, slightly damaged
            if (fuel >= getInitialFuel()*0.33)
                return 6;        // extinguished, more damaged
            if (fuel>0)
                return 7;        // extinguished, severely damaged
            return 8;           // completely burnt down
        }
        
        public double getRadiationEnergy(){
            double t = getTemperature() + 293; // Assume ambient temperature is 293 Kelvin.
            double radEn = (t*t*t*t)*totalWallArea*RADIATION_COEFFICIENT*STEFAN_BOLTZMANN_CONSTANT;
            if (radEn == Double.NaN || radEn == Double.POSITIVE_INFINITY || radEn == Double.NEGATIVE_INFINITY)
                radEn = Double.MAX_VALUE*0.75;
            if (radEn>getEnergy()){
                radEn = getEnergy();
            }
            return radEn;
        }
        
        public int getRealFieryness(){
            return selfBuilding.getFieryness();
        }
        
        public int getRealTemperature(){
            return selfBuilding.getTemperature();
        }
        
        public Building getSelfBuilding(){
            return selfBuilding;
        }
        
        public float getVolume(){
            return volume;
        }
        
        public float getCapacity(){
            return capacity;
        }
        
        public double getEnergy(){
            return energy;
        }
        
        public void setEnergy(double v){
            energy = v;
        }
        
        public float getPrevBurned(){
            return prevBurned;
        }
        
        public void setPrevBurned(float consumed){
            prevBurned = consumed;
        }
        
        public boolean isFlammable(){
            return flammable;
        }
        
        public void setFlammable(boolean flammable){
            this.flammable = flammable;
        }
        
        public float getFuel(){
            return fuel;
        }
        
        public void setFuel(float fuel){
            this.fuel = fuel;
        }
        
        public int getWaterQuantity(){
            return waterQuantity;
        }
        
        public void setWaterQuantity(int i){
            if (i>waterQuantity){
                wasEverWatered = true;
            }
            waterQuantity = i;
        }
        
        public void increaseWaterQuantity(int i){
            waterQuantity += i;
        }
        
        public int getStartTime(){
            return startTime;
        }
        
        public void setStartTime(int startTime){
            this.startTime = startTime;
        }
        
        public void setWasEverWatered(boolean wasEverWatered){
            this.wasEverWatered = wasEverWatered;
        }
        
        public boolean isPutOff(){
            return getFieryness()>4 && getFieryness()<8;
        }
        
        public boolean isBurned(){
            return getFieryness() == 8;
        }
        
        public EntityID getID(){
            return selfBuilding.getID();
        }
        
    }
    
    public class Wall {
        public int MAX_SAMPLE_DISTANCE = 50000;
        public int x1;
        public int y1;
        public int x2;
        public int y2;
        public RIOBuilding owner;
        public int rays;
        public int hits;
        public int selfHits;
        public int strange;
        public double length;
        public Point a;
        public Point b;
        
        public int distance;
        
        public Wall(int x1, int y1, int x2, int y2, RIOBuilding owner, float rayRate) {
            this.x1 = x1;
            this.y1 = y1;
            this.x2 = x2;
            this.y2 = y2;
            a = new Point(x1, y1);
            b = new Point(x2, y2);
            length = a.distance(b);
            rays = (int) Math.ceil(length * rayRate);
            hits = 0;
            this.owner = owner;
        }
    
        public void findHits(World world) {
            selfHits=0;
            strange=0;
            for(int emitted=0;emitted<rays;emitted++){
                //creating ray
                Point start=firesimulator.util.Geometry.getRndPoint(a,b);
                if(start==null){
                    strange++;
                    continue;
                }
                Point end=firesimulator.util.Geometry.getRndPoint(start,MAX_SAMPLE_DISTANCE);
                //intersect
                Wall closest=null;
                double minDist=Double.MAX_VALUE;
                for(Iterator it=simulator.allWalls.iterator();it.hasNext();){
                    Wall other=(Wall)it.next();
                    if(other==this) continue;
                    Point cross=firesimulator.util.Geometry.intersect(start,end,other.a,other.b);
                    if(cross!=null){
                        if(cross.distance(start)<minDist){
                            minDist=cross.distance(start);
                            closest=other;
                        }
                    }
                }
                if(closest == null){
                    //Nothing was hit
                    continue;
                }
                if(closest.owner==this.owner){
                    //The source building was hit
                    selfHits++;
                }
                if(closest!=this&&closest!=null&&closest.owner!=owner){
                    hits++;
                    Integer value=(Integer)owner.connectedBuildings.get(closest.owner);
                    int temp = 0;
                    if(value != null){
                        temp = value.intValue();
                    }
                    temp++;
                    owner.connectedBuildings.put(closest.owner,new Integer(temp));
                }
            }
        }
        
        public boolean validate() {
            return !(a.x == b.x && a.y == b.y);
        }
        
    }
    
    public class RIOBuildingConnection {
        public int toID = 0;
        public float weight = 0;
        
        public RIOBuildingConnection(int toID, float weight) {
            this.toID = toID;
            this.weight = weight;
        }
    }
    
}
