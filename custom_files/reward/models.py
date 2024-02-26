from typing import List, Tuple

from pydantic import confloat, conint


DistanceFromCenter = confloat(ge=0)
TrackWidth = confloat(ge=0)
TrackLength = confloat(ge=0)
Index = conint(ge=0)
ClosestWaypoints = List[conint(ge=0)]
Waypoint = Tuple[float, float]
Waypoints = List[Waypoint]
Speed = confloat(ge=0, le=4)
SteeringAngle = confloat(ge=-30, le=30)
Progress = confloat(ge=0, le=100)
Steps = conint(ge=0)
Heading = confloat(ge=-180, le=180)
Heading360 = confloat(ge=0, le=360)
Percentage = confloat(ge=0, le=1)
Distance = confloat(ge=0)

