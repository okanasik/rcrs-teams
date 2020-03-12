package RIO_2019.centralized;


import adf.agent.action.common.ActionMove;
import adf.agent.communication.MessageManager;
import adf.agent.communication.standard.bundle.centralized.CommandPolice;
import adf.agent.communication.standard.bundle.centralized.CommandScout;
import adf.agent.communication.standard.bundle.centralized.MessageReport;
import adf.agent.develop.DevelopData;
import adf.agent.info.AgentInfo;
import adf.agent.info.ScenarioInfo;
import adf.agent.info.WorldInfo;
import adf.agent.module.ModuleManager;
import adf.agent.precompute.PrecomputeData;
import adf.component.centralized.CommandExecutor;
import adf.component.module.algorithm.PathPlanning;
import rescuecore2.standard.entities.Area;
import rescuecore2.standard.entities.Blockade;
import rescuecore2.standard.entities.Human;
import rescuecore2.standard.entities.Road;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.worldmodel.AbstractEntity;
import rescuecore2.worldmodel.EntityID;

import java.awt.Polygon;
import java.awt.Shape;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

import static rescuecore2.standard.entities.StandardEntityURN.AMBULANCE_TEAM;
import static rescuecore2.standard.entities.StandardEntityURN.FIRE_BRIGADE;
import static rescuecore2.standard.entities.StandardEntityURN.REFUGE;

public class RIOCommandExecutorScout extends CommandExecutor<CommandScout> {
	private static final int ACTION_UNKNOWN = -1;
	private static final int ACTION_SCOUT = 1;

	private int sendingAvoidTimeClearRequest;
	private int lastSentTime;

	private PathPlanning pathPlanning;

	private int type;
	private Collection<EntityID> scoutTargets;
	private EntityID commanderID;

	public RIOCommandExecutorScout(AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager, DevelopData developData) {
		super(ai, wi, si, moduleManager, developData);
		this.sendingAvoidTimeClearRequest = developData.getInteger("sample.tactics.MessageTool.sendingAvoidTimeClearRequest", 5);
		this.lastSentTime=0;
		this.type = ACTION_UNKNOWN;
		switch  (scenarioInfo.getMode()) {
		case PRECOMPUTATION_PHASE:
			this.pathPlanning = moduleManager.getModule("CommandExecutorScout.PathPlanning", "RIO_2019.module.algorithm.AstarPathPlanning");
			break;
		case PRECOMPUTED:
			this.pathPlanning = moduleManager.getModule("CommandExecutorScout.PathPlanning", "RIO_2019.module.algorithm.AstarPathPlanning");
			break;
		case NON_PRECOMPUTE:
			this.pathPlanning = moduleManager.getModule("CommandExecutorScout.PathPlanning", "RIO_2019.module.algorithm.AstarPathPlanning");
			break;
		}
	}

	@Override
	public CommandExecutor setCommand(CommandScout command) {
		/*EntityID agentID = this.agentInfo.getID();
        if(command.isToIDDefined() && (Objects.requireNonNull(command.getToID()).getValue() == agentID.getValue())) {
            EntityID target = command.getTargetID();
            if(target == null) {
                target = this.agentInfo.getPosition();
            }
            this.type = ACTION_SCOUT;
            this.commanderID = command.getSenderID();
            this.scoutTargets = new HashSet<>();
            this.scoutTargets.addAll(
                    worldInfo.getObjectsInRange(target, command.getRange())
                            .stream()
                            .filter(e -> e instanceof Area && e.getStandardURN() != REFUGE)
                            .map(AbstractEntity::getID)
                            .collect(Collectors.toList())
            );
        }*/
		return this;
	}

	@Override
	public CommandExecutor updateInfo(MessageManager messageManager){
		super.updateInfo(messageManager);
		sendRequestMessagesInExtendedBlockades(agentInfo, worldInfo, scenarioInfo, messageManager);
		if(this.getCountUpdateInfo() >= 2) {
			return this;
		}
		this.pathPlanning.updateInfo(messageManager);

		if(this.isCommandCompleted()) {
			if(this.type != ACTION_UNKNOWN) {
				messageManager.addMessage(new MessageReport(true, true, false, this.commanderID));
				this.type = ACTION_UNKNOWN;
				this.scoutTargets = null;
				this.commanderID = null;
			}
		}
		return this;
	}

	private void sendRequestMessagesInExtendedBlockades (AgentInfo agentInfo, WorldInfo worldInfo, ScenarioInfo scenarioInfo, MessageManager messageManager)
	{
		if (agentInfo.me().getStandardURN() == AMBULANCE_TEAM
				|| agentInfo.me().getStandardURN() == FIRE_BRIGADE)
		{
			int currentTime = agentInfo.getTime();
			Human agent = (Human) agentInfo.me();
			if (isInExtendedBlockade(agent) && ((currentTime - this.lastSentTime) >= this.sendingAvoidTimeClearRequest))
			{
				this.lastSentTime = currentTime;
				messageManager.addMessage(
						new CommandPolice( true, null, agent.getPosition(), CommandPolice.ACTION_CLEAR )
						);
			}

		}
	}

	private boolean isInExtendedBlockade(Human human) {
		if(!human.isXDefined() || !human.isXDefined()) return false;
		int agentX = human.getX();
		int agentY = human.getY();
		StandardEntity positionEntity = this.worldInfo.getPosition(human);
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
				xs[i] += (vectorX / magnitude) * 2 + 0.5;
				ys[i] += (vectorY / magnitude) * 2 + 0.5;
			}
			shape = new Polygon(xs, ys, count);
		}
		return shape;
	}

	@Override
	public CommandExecutor precompute(PrecomputeData precomputeData) {
		super.precompute(precomputeData);
		if(this.getCountPrecompute() >= 2) {
			return this;
		}
		this.pathPlanning.precompute(precomputeData);
		return this;
	}

	@Override
	public CommandExecutor resume(PrecomputeData precomputeData) {
		super.resume(precomputeData);
		if(this.getCountResume() >= 2) {
			return this;
		}
		this.pathPlanning.resume(precomputeData);
		return this;
	}

	@Override
	public CommandExecutor preparate() {
		super.preparate();
		if(this.getCountPreparate() >= 2) {
			return this;
		}
		this.pathPlanning.preparate();
		return this;
	}

	@Override
	public CommandExecutor calc() {
		this.result = null;
		/*if(this.type == ACTION_SCOUT) {
            if(this.scoutTargets == null || this.scoutTargets.isEmpty()) {
                return this;
            }
            this.pathPlanning.setFrom(this.agentInfo.getPosition());
            this.pathPlanning.setDestination(this.scoutTargets);
            List<EntityID> path = this.pathPlanning.calc().getResult();
            if(path != null) {
                this.result = new ActionMove(path);
            }
        }*/
		return this;
	}

	private boolean isCommandCompleted() {
		if(this.type ==  ACTION_SCOUT) {
			if(this.scoutTargets != null) {
				this.scoutTargets.removeAll(this.worldInfo.getChanged().getChangedEntities());
			}
			return (this.scoutTargets == null || this.scoutTargets.isEmpty());
		}
		return true;
	}
}