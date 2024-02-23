import pydantic
from pydantic import confloat, RootModel


print(pydantic.__version__)

SubReward = RootModel[confloat(ge=0, le=1)]

