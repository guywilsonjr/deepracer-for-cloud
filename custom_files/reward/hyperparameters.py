from pydantic import BaseModel, Field
from typing_extensions import Literal


class Hyperparameters(BaseModel):
    batch_size: Literal[32, 64, 128, 256, 512]
    beta_entropy: float = Field(ge=0, le=1)
    discount_factor: float = Field(ge=0, le=1)
    e_greedy_value: float = Field(ge=0, le=1)
    epsilon_steps: int = Field(ge=0)
    exploration_type: Literal['e-greedy', 'categorical']
    loss_type: Literal['mse', 'huber']
    lr: float = Field(ge=1e-8, le=1e-3)
    num_episodes_between_training: int = Field(ge=5, le=100)
    term_cond_avg_score: float = Field(ge=0)
    term_cond_max_episodes: int = Field(ge=0)
    # Uncertain what this does
    stack_size: int