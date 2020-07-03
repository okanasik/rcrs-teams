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

public class RLBuildingDetector extends BuildingDetector {
    private EntityID result = null;
    private RPCClient rpcClient;
    private SingleFrame singleFrame;

    public RLBuildingDetector(AgentInfo ai, WorldInfo wi, ScenarioInfo si,
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
    public RLBuildingDetector precompute(PrecomputeData precomputeData) {
        super.precompute(precomputeData);
        if (this.getCountPrecompute() > 1) {
            return this;
        }
        return this;
    }

    @Override
    public RLBuildingDetector resume(PrecomputeData precomputeData) {
        super.resume(precomputeData);
        if (this.getCountResume() > 1) {
            return this;
        }
        this.preparate();
        return this;
    }

    @Override
    public RLBuildingDetector preparate() {
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
    public RLBuildingDetector updateInfo(MessageManager messageManager) {
        super.updateInfo(messageManager);
        if (this.getCountUpdateInfo() > 1) {
            return this;
        }
        return this;
    }

    @Override
    public RLBuildingDetector calc() {
        singleFrame.update(worldInfo, agentInfo);
        Gson gson = new GsonBuilder().create();
        String episodeJsonString = gson.toJson(singleFrame);
        try {
            String targetBuilding = rpcClient.call("observation_rl", episodeJsonString);
            result = new EntityID(Integer.parseInt(targetBuilding));
        } catch (Exception ex) {
            try {
                rpcClient.close();
            } catch (Exception ex2) {
                ex2.printStackTrace();
            }
            ex.printStackTrace();
            System.exit(200);
        }

        if (agentInfo.getTime() == scenarioInfo.getKernelTimesteps()) {
            try {
                rpcClient.close();
            } catch (Exception ex) {
                ex.printStackTrace();
                System.exit(300);
            }
        }
        return this;
    }

}

