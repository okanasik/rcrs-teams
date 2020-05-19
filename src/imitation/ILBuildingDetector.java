package imitation;

import adf.agent.communication.MessageManager;
import adf.agent.develop.DevelopData;
import adf.agent.info.AgentInfo;
import adf.agent.info.ScenarioInfo;
import adf.agent.info.WorldInfo;
import adf.agent.module.ModuleManager;
import adf.agent.precompute.PrecomputeData;
import adf.component.module.complex.BuildingDetector;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import conn.RPCClient;
import data.SingleFrame;
import rescuecore2.worldmodel.EntityID;

public class ILBuildingDetector extends BuildingDetector {
    private EntityID result = null;
    private RPCClient rpcClient;
    private SingleFrame singleFrame;

    public ILBuildingDetector(AgentInfo ai, WorldInfo wi, ScenarioInfo si,
                              ModuleManager mm, DevelopData dd) {
        super(ai, wi, si, mm, dd);
        rpcClient = new RPCClient();
        try {
            rpcClient.connect("localhost");
        } catch (Exception ex) {
            ex.printStackTrace();
            System.exit(100);
        }
        singleFrame = new SingleFrame();
    }

    @Override
    public ILBuildingDetector precompute(PrecomputeData precomputeData) {
        super.precompute(precomputeData);
        if (this.getCountPrecompute() > 1) {
            return this;
        }
        return this;
    }

    @Override
    public ILBuildingDetector resume(PrecomputeData precomputeData) {
        super.resume(precomputeData);
        if (this.getCountResume() > 1) {
            return this;
        }
        this.preparate();
        return this;
    }

    @Override
    public ILBuildingDetector preparate() {
        super.preparate();
        if (this.getCountPreparate() > 1) {
            return this;
        }
        return this;
    }

    @Override
    public EntityID getTarget() {
        return this.result;
    }

    @Override
    public ILBuildingDetector updateInfo(MessageManager messageManager) {
        super.updateInfo(messageManager);
        if (this.getCountUpdateInfo() > 1) {
            return this;
        }
        return this;
    }

    @Override
    public ILBuildingDetector calc() {
        singleFrame.update(worldInfo, agentInfo);
        Gson gson = new GsonBuilder().setPrettyPrinting().create();
        String episodeJsonString = gson.toJson(singleFrame);
        try {
            String targetBuilding = rpcClient.call("building_detector", episodeJsonString);
            result = new EntityID(Integer.parseInt(targetBuilding));
        } catch (Exception ex) {
            ex.printStackTrace();
            System.exit(200);
        }
        return this;
    }

}

