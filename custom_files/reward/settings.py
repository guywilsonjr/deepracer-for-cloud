from pydantic import confloat

from .constants import MAX_CONFIG_SPEED, MAX_CONFIG_STEERING, MIN_CONFIG_SPEED, MIN_CONFIG_STEERING


SpeedSetting = confloat(ge=MIN_CONFIG_SPEED, le=MAX_CONFIG_SPEED)
SteeringAngleSetting = confloat(ge=MIN_CONFIG_STEERING, le=MAX_CONFIG_STEERING)