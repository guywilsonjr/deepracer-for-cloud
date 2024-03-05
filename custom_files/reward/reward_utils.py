from pydantic import BaseModel, confloat, model_validator

from .constants import MIN_REWARD


class SubReward(BaseModel):
    reward: confloat(ge=0, le=1)

    @model_validator(mode='after')
    def model_validator(self):
        self.reward = max(self.reward, MIN_REWARD)


