package AIT_2019.module.complex;

import static rescuecore2.standard.entities.StandardEntityURN.AMBULANCE_TEAM;
import static rescuecore2.standard.entities.StandardEntityURN.FIRE_BRIGADE;
import static rescuecore2.standard.entities.StandardEntityURN.POLICE_FORCE;

import java.awt.Shape;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;
import java.util.stream.Collectors;

import com.mrl.debugger.remote.VDClient;

import adf.agent.communication.MessageManager;
import adf.agent.communication.standard.bundle.MessageUtil;
import adf.agent.communication.standard.bundle.StandardMessagePriority;
import adf.agent.communication.standard.bundle.centralized.CommandPolice;
import adf.agent.communication.standard.bundle.information.MessageBuilding;
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
import AIT_2019.module.algorithm.ConvexHull;
import AIT_2019.module.algorithm.StuckedHumans;
import rescuecore2.standard.entities.Area;
import rescuecore2.standard.entities.Building;
import rescuecore2.standard.entities.Human;
import rescuecore2.standard.entities.Refuge;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.worldmodel.EntityID;

public class AITBuildingSearch extends Search
{
    private PathPlanning pathPlanning;
    private Clustering clustering;
    private Clustering stuckedHumans;

    private int avoidTimeSendingReceived = -1;
    private int avoidTimeSendingSent = -1;
    private Map<EntityID, Integer> sentTimeMap = new HashMap<>();

    private Random random = new Random();

    private boolean hasFocusedAssignedCluster = false;
    private List<EntityID> buildingIDsOfFocusedCluster = new ArrayList<>();
    private List<Integer> indexOfEverFocusedClusters = new ArrayList<>();

    private EntityID targetID = null;
    private EntityID result = null;

    // Debug
    // private VDClient vdclient = VDClient.getInstance();
    // /Debug

    public AITBuildingSearch(AgentInfo ai, WorldInfo wi, ScenarioInfo si,
            ModuleManager mm, DevelopData dd)
    {
        super(ai, wi, si, mm, dd);
        this.pathPlanning = moduleManager.getModule(
                "SampleSearch.PathPlaning.Fire",
                "adf.sample.module.algorithm.SamplePathPlanning");
        this.clustering = moduleManager.getModule(
                "SampleSearch.Clustering.Fire",
                "adf.sample.module.algorithm.SampleKMeans");
        this.stuckedHumans = moduleManager.getModule(
                "AITActionExtClear.StuckedHumans",
                "AIT_2019.module.algorithm.StuckedHumans");
        this.registerModule(this.pathPlanning);
        this.registerModule(this.clustering);
        this.registerModule(this.stuckedHumans);
        this.avoidTimeSendingReceived = 4;
        this.avoidTimeSendingSent = 3;
        this.random.setSeed(ai.getID().getValue());

        // Debug
        // this.vdclient.init("localhost", 1099);
        // /Debug
    }

    @Override
    public Search precompute(PrecomputeData precomputeData)
    {
        super.precompute(precomputeData);
        if (this.getCountPrecompute() > 1)
        {
            return this;
        }
        return this;
    }

    @Override
    public Search resume(PrecomputeData precomputeData)
    {
        super.resume(precomputeData);
        if (this.getCountResume() > 1)
        {
            return this;
        }
        this.preparate();
        return this;
    }

    @Override
    public EntityID getTarget()
    {
        return this.result;
    }

    @Override
    public Search preparate()
    {
        super.preparate();
        if (this.getCountPreparate() > 1)
        {
            return this;
        }

        this.pathPlanning.preparate();
        this.clustering.preparate();
        this.stuckedHumans.preparate();
        return this;
    }

    @Override
    public Search updateInfo(MessageManager messageManager)
    {
        super.updateInfo(messageManager);
        if (this.getCountUpdateInfo() > 1)
        {
            return this;
        }

        this.pathPlanning.updateInfo(messageManager);
        this.clustering.updateInfo(messageManager);
        this.stuckedHumans.updateInfo(messageManager);
        this.sendChangedEntityInfo(messageManager);
        this.reflectOtherEntityInfo(messageManager);

        if (this.agentInfo.getTime() < 1)
        {
            return this;
        }
        if (this.isStuckedInBlockade())
        {
            messageManager.addMessage(new CommandPolice(
                    true, StandardMessagePriority.HIGH, null,
                    this.agentInfo.getPosition(), CommandPolice.ACTION_CLEAR));
            messageManager.addMessage(new CommandPolice(
                    false, StandardMessagePriority.HIGH, null,
                    this.agentInfo.getPosition(), CommandPolice.ACTION_CLEAR));
        }
        if(this.buildingIDsOfFocusedCluster.isEmpty())
        {
            this.setFocusedCluster();
        }

        List<EntityID> changedEntityIDs = this.worldInfo.getChanged().getChangedEntities().stream()
                .map(id -> this.worldInfo.getEntity(id))
                .filter(Building.class::isInstance)
                .filter(se -> !(se instanceof Refuge))
                .map(StandardEntity::getID)
                .collect(Collectors.toList());
        this.buildingIDsOfFocusedCluster.removeAll(changedEntityIDs);

        if(this.buildingIDsOfFocusedCluster.isEmpty())
        {
            this.setFocusedCluster();
        }

        // Debug
        // if (this.agentInfo.me().getStandardURN() != FIRE_BRIGADE) { return this; }
        // List<Shape> datas = new ArrayList<>();
        // for (EntityID id : this.buildingIDsOfFocusedCluster)
        // {
            // StandardEntity entity = this.worldInfo.getEntity(id);
            // if (!(entity instanceof Area)) { continue; }
            // Area area = (Area) entity;
            // datas.add(area.getShape());
        // }
        // this.vdclient.drawAsync(
            // this.agentInfo.getID().getValue(),
            // "SamplePolygon",
            // (Serializable) datas);
        // /Debug

        return this;
    }

    @Override
    public Search calc()
    {
        if (this.agentInfo.getTime() < 1)
        {
            this.clustering.calc();
            return this;
        }
        if (this.isStuckedInBlockade())
        {
            return this;
        }

        // Debug
        // if (this.result != null)
        // {
            // Area area = (Area) this.worldInfo.getEntity(this.result);
            // this.vdclient.drawAsync(
                    // this.agentInfo.getID().getValue(),
                    // "ClusterConvexhull",
                    // (Serializable) Arrays.asList(area.getShape()));
        // }
        // /Debug

        if (this.targetID != null
                && this.buildingIDsOfFocusedCluster.contains(this.targetID))
        {
            return this;
        }

        this.result = null;
        this.targetID = null;
        int size = this.buildingIDsOfFocusedCluster.size();
        if (size <= 0) {
            targetID = this.agentInfo.getPosition();
            result = targetID; // stay at the current position
            return this;
        }
        int index = this.random.nextInt(size);
        this.targetID = this.buildingIDsOfFocusedCluster.get(index);
        this.pathPlanning.setFrom(this.agentInfo.getPosition());
        this.pathPlanning.setDestination(targetID);
        List<EntityID> path = this.pathPlanning.calc().getResult();

        if (path != null && path.size() > 0)
        {
            StandardEntity entity = this.worldInfo.getEntity(path.get(path.size() - 1));
            if (entity instanceof Building) { path.remove(path.size() - 1); }
            if (path.size() > 1) {
                this.result = path.get(path.size() - 1);
            } else {
                // set the start position
                this.result = agentInfo.getPosition();
            }
        }
        this.out("SEARCH #" +  this.result);
        return this;
    }


    private void setFocusedCluster()
    {
        int clusterIndex = -1;
        this.clustering.calc();
        if (!this.hasFocusedAssignedCluster)
        {
            clusterIndex = this.clustering.getClusterIndex(this.agentInfo.getID());
            this.indexOfEverFocusedClusters.add(clusterIndex);
            this.hasFocusedAssignedCluster = true;
        }
        else
        {
            int clusterNumber = this.clustering.getClusterNumber();
            if (this.indexOfEverFocusedClusters.size() == clusterNumber) {
                this.indexOfEverFocusedClusters.clear();
                this.hasFocusedAssignedCluster = false;
            }
            clusterIndex = this.random.nextInt(clusterNumber);

            while (this.indexOfEverFocusedClusters.contains(clusterIndex))
            {
                clusterIndex = this.random.nextInt(clusterNumber);
            }
        }
        this.buildingIDsOfFocusedCluster = this.clustering.getClusterEntities(clusterIndex).stream()
                .filter(Building.class::isInstance)
                .filter(se -> !(se instanceof Refuge))
                .map(StandardEntity::getID)
                .collect(Collectors.toList());

        // Debug
        // if (this.agentInfo.me().getStandardURN() != FIRE_BRIGADE) { return; }
        // List<Shape> datas = new ArrayList<>();
        // for (StandardEntity entity : this.clustering.getClusterEntities(clusterIndex))
        // {
            // if (!(entity instanceof Area)) { continue; }
            // Area area = (Area) entity;
            // datas.add(area.getShape());
        // }
        // this.vdclient.drawAsync(
            // this.agentInfo.getID().getValue(),
            // "ClusterConvexhull",
            // (Serializable) datas);
        // /Debug
    }

    private Boolean checkShouldSend()
    {
        boolean shouldSendMessage = true;
        StandardEntity agentMe = this.agentInfo.me();
        Human me = (Human) agentMe;
        Collection<StandardEntity> agents = this.worldInfo.getEntitiesOfType(
                AMBULANCE_TEAM, FIRE_BRIGADE, POLICE_FORCE);
        agents.remove(agentMe);
        for (StandardEntity agent : agents)
        {
            if (!shouldSendMessage) { break; }
            if (!(agent instanceof Human)) { continue; }

            Human other = (Human) agent;
            if (other.getPosition() != me.getPosition()) { continue; }
            if (other.getID().getValue() > me.getID().getValue())
            {
                shouldSendMessage = false;
            }
        }
        return shouldSendMessage;
    }

    private Building selectPreferred(Building bld1, Building bld2)
    {
        if (bld1 == null && bld2 == null) { return null; }
        else if (bld1 != null && bld2 == null) { return bld1; }
        else if (bld1 == null && bld2 != null) { return bld2; }

        if (bld1.isOnFire() && bld2.isOnFire())
        {
            if (bld1.isFierynessDefined() && bld2.isFierynessDefined())
            {
                return (bld1.getFieryness() > bld2.getFieryness()) ? bld1 : bld2;
            }
            if (bld1.isTemperatureDefined() && bld2.isTemperatureDefined())
            {
                return (bld1.getTemperature() > bld2.getTemperature()) ? bld1 : bld2;
            }
        }
        else if (bld1.isOnFire() && !bld2.isOnFire())
        {
            return bld1;
        }

        return bld2;
    }

    private void sendChangedEntityInfo(MessageManager messageManager)
    {
        if (!this.checkShouldSend()) { return; }

        Building building = null;
        int currTime = this.agentInfo.getTime();
        Human me = (Human) this.agentInfo.me();
        List<EntityID> agentPositions = this.worldInfo.getEntitiesOfType(
                AMBULANCE_TEAM, FIRE_BRIGADE, POLICE_FORCE).stream()
            .map(Human.class::cast)
            .map(Human::getPosition)
            .collect(Collectors.toList());
        for (EntityID id : this.worldInfo.getChanged().getChangedEntities())
        {
            Integer time = this.sentTimeMap.get(id);
            if (time != null && time > currTime) { continue; }

            StandardEntity entity = this.worldInfo.getEntity(id);
            if (!(entity instanceof Building)) { continue; }
            Building bld = (Building) entity;
            if (!agentPositions.contains(bld.getID())
                    || bld.getID().equals(me.getPosition()))
            {
                building = this.selectPreferred(building, bld);
            }
        }

        if (building != null)
        {
            messageManager.addMessage(new MessageBuilding(true, building));
            this.sentTimeMap.put(building.getID(), currTime + this.avoidTimeSendingSent);
            this.out("SEND #" + building.getID());
        }
    }

    private void reflectOtherEntityInfo(MessageManager messageManager)
    {
        Set<EntityID> changedEntityIDs =
                this.worldInfo.getChanged().getChangedEntities();
        int time = this.agentInfo.getTime();
        for (CommunicationMessage message
                : messageManager.getReceivedMessageList(MessageBuilding.class))
        {
            MessageBuilding msg = (MessageBuilding) message;
            if (!changedEntityIDs.contains(msg.getBuildingID()))
            {
                MessageUtil.reflectMessage(this.worldInfo, msg);
            }
            this.sentTimeMap.put(msg.getBuildingID(), time + this.avoidTimeSendingReceived);
        }
    }

    private boolean isStuckedInBlockade()
    {
        return this.stuckedHumans.calc().getClusterIndex(this.agentInfo.getID()) == 0;
    }

    private void out(String str)
    {
        String ret;
        ret  = "🚒  [" + String.format("%10d", this.agentInfo.getID().getValue())+ "]";
        ret += " BUILDING-SEARCH ";
        ret += "@" + String.format("%3d", this.agentInfo.getTime());
        ret += " -> ";
//        System.out.println(ret + str);
    }
}
