# ---------------------------------------------------------------------------
# Append this block to openpi/src/openpi/training/config.py (before _CONFIGS)
# after copying u0bot_policy.py to openpi/src/openpi/policies/.
#
# Adapted from VincentGu2000/u0-openpi (Apache-2.0), which fine-tunes pi0.5 on
# the USIM dataset. The block consumes our LeRobot v2.1 dataset unchanged.
# ---------------------------------------------------------------------------
# Required imports are already present in config.py:
#   import dataclasses, os, pathlib
#   from typing_extensions import override
#   import flax.nnx as nnx
#   import openpi.models.model as _model
#   import openpi.models.pi0_config as pi0_config
#   import openpi.policies.u0bot_policy as u0bot_policy
#   import openpi.shared.nnx_utils as nnx_utils
#   import openpi.training.optimizer as _optimizer
#   import openpi.training.weight_loaders as weight_loaders
#   import openpi.transforms as _transforms


@dataclasses.dataclass(frozen=True)
class LeRobotU0BotDataConfig(DataConfigFactory):
    """LeRobot v2.1 data config for the USIM/U0 underwater robot.

    Expected keys in each episode:
      observation.images.ego    (T, 240, 320, 3) video
      observation.images.wrist  (T, 240, 320, 3) video
      observation.state         (T, 29) float32
      action                    (T, 13) float32
      target_pos                (T, 6)  float32 (optional for pi0.5)
    """

    @override
    def create(self, assets_dirs: pathlib.Path, model_config: _model.BaseModelConfig) -> DataConfig:
        repack_transform = _transforms.Group(
            inputs=[
                _transforms.RepackTransform(
                    {
                        "observation/ego_image": "observation.images.ego",
                        "observation/wrist_image": "observation.images.wrist",
                        "observation/state": "observation.state",
                        "actions": "action",
                        "prompt": "prompt",
                    }
                )
            ]
        )
        data_transforms = _transforms.Group(
            inputs=[u0bot_policy.U0BotInputs(model_type=model_config.model_type)],
            outputs=[u0bot_policy.U0BotOutputs()],
        )
        model_transforms = ModelTransformFactory()(model_config)
        return dataclasses.replace(
            self.create_base_config(assets_dirs, model_config),
            repack_transforms=repack_transform,
            data_transforms=data_transforms,
            model_transforms=model_transforms,
            action_sequence_keys=("action",),
        )


# Add this TrainConfig to the _CONFIGS list.
TRAIN_CONFIG_SNIPPET = TrainConfig(
    name="pi05_uuv_charge_nav",
    model=pi0_config.Pi0Config(
        pi05=True,
        action_horizon=16,
        discrete_state_input=False,
    ),
    data=LeRobotU0BotDataConfig(
        repo_id=os.path.join(
            os.path.expanduser(os.environ.get("DATA_BASE_DIR", "~/data")),
            "usim",
            "train",
        ),
        base_config=DataConfig(prompt_from_task=True),
    ),
    weight_loader=weight_loaders.CheckpointWeightLoader(
        "gs://openpi-assets/checkpoints/pi05_base/params"
    ),
    # Freeze PaliGemma LLM; train vision tower + action expert.
    freeze_filter=nnx.All(
        nnx_utils.PathRegex(".*llm.*"),
        nnx.Not(nnx_utils.PathRegex(".*llm.*_1.*")),
    ),
    batch_size=64,
    lr_schedule=_optimizer.CosineDecaySchedule(
        warmup_steps=1000,
        peak_lr=5e-5,
        decay_steps=100000,
        decay_lr=5e-5,
    ),
    optimizer=_optimizer.AdamW(clip_gradient_norm=1.0),
    ema_decay=0.999,
    num_train_steps=22000,
    save_interval=11000,
    keep_period=11000,
)
