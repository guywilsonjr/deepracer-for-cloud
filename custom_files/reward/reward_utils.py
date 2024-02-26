from pydantic import confloat, RootModel


SubReward = RootModel[confloat(ge=0, le=1)]

