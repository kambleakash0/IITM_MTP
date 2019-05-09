from __future__ import absolute_import
from __future__ import print_function

# import os
# import sys
# import optparse
# import subprocess
import random

random.seed(42)  # make tests reproducible
N = 20000  # number of time steps, initially 3600
# demand per second from different directions
p0 = 1. / 30
p1 = 1. / 20
p4 = 1. / 20
p9 = 1. / 30
p10 = 1. / 30
p15 = 1. / 20
p18 = 1. / 20
p19 = 1. / 30

# Route template
# <route id="4_0" edges="" />
# <route id="4_10" edges="" />
# <route id="4_18" edges="" />
# <route id="4_19" edges="" />
# <route id="4_19" edges="" />
# <route id="4_15" edges="" />
# <route id="4_15" edges="" />
# <route id="4_9" edges="" />
# <route id="4_1" edges="" />

with open("data/2x2.rou.xml", "w") as routes:
    print("""<routes>

    <vType id="type0" accel="0.8" decel="4.5" sigma="0.2" length="5" minGap="2.5" maxSpeed="25" guiShape="passenger"/>
    <vType id="type1" accel="0.8" decel="4.5" sigma="0.2" length="5" minGap="2.5" maxSpeed="25" guiShape="passenger"/>
    <vType id="type4" accel="0.8" decel="4.5" sigma="0.2" length="5" minGap="2.5" maxSpeed="25" guiShape="passenger"/>
    <vType id="type9" accel="0.8" decel="4.5" sigma="0.2" length="5" minGap="2.5" maxSpeed="25" guiShape="passenger"/>
    <vType id="type10" accel="0.8" decel="4.5" sigma="0.2" length="5" minGap="2.5" maxSpeed="25" guiShape="passenger"/>
    <vType id="type15" accel="0.8" decel="4.5" sigma="0.2" length="5" minGap="2.5" maxSpeed="25" guiShape="passenger"/>
    <vType id="type18" accel="0.8" decel="4.5" sigma="0.2" length="5" minGap="2.5" maxSpeed="25" guiShape="passenger"/>
    <vType id="type19" accel="0.8" decel="4.5" sigma="0.2" length="5" minGap="2.5" maxSpeed="25" guiShape="passenger"/>

    <route id="0_4" edges="0-2 2-6 6-5 5-4" />
    <route id="0_10" edges="0-2 2-6 6-12 12-11 11-10" />
    <route id="0_18" edges="0-2 2-6 6-12 12-16 16-18" />

    <route id="4_10" edges="4-5 5-6 6-12 12-11 11-10" />
    <route id="4_18" edges="4-5 5-6 6-12 12-16 16-18" />
    <route id="4_19" edges="4-5 5-6 6-7 7-13 13-17 17-19" />
    <route id="4_9" edges="4-5 5-6 6-7 7-8 8-9" />

    <route id="10_18" edges="10-11 11-12 12-16 16-18" />
    <route id="10_19" edges="10-11 11-12 12-13 13-17 17-19" />
    <route id="10_15" edges="10-11 11-12 12-13 13-14 14-15" />

    <route id="18_0" edges="18-16 16-12 12-6 6-2 2-0" />
    <route id="18_19" edges="18-16 16-12 12-13 13-17 17-19" />
    <route id="18_15" edges="18-16 16-12 12-13 13-14 14-15" />
    <route id="18_9" edges="18-16 16-12 12-6 6-7 7-8 8-9" />

    <route id="19_15" edges="19-17 17-13 13-14 14-15" />
    <route id="19_9" edges="19-17 17-13 13-7 7-8 8-9" />
    <route id="19_1" edges="19-17 17-13 13-7 7-3 3-1" />

    <route id="15_0" edges="15-14 14-13 13-12 12-6 6-2 2-0" />
    <route id="15_10" edges="15-14 14-13 13-12 12-11 11-10" />
    <route id="15_9" edges="15-14 14-13 13-7 7-8 8-9" />
    <route id="15_1" edges="15-14 14-13 13-7 7-3 3-1" />

    <route id="9_0" edges="9-8 8-7 7-6 6-2 2-0" />
    <route id="9_4" edges="9-8 8-7 7-6 6-5 5-4" />
    <route id="9_1" edges="9-8 8-7 7-3 3-1" />

    <route id="1_0" edges="1-3 3-7 7-6 6-2 2-0" />
    <route id="1_4" edges="1-3 3-7 7-6 6-5 5-4" />
    <route id="1_10" edges="1-3 3-7 7-13 13-12 12-11 11-10" />
    <route id="1_19" edges="1-3 3-7 7-13 13-17 17-19" />
    """, file=routes)
    lastVeh = 0
    vehNr = 0
    for i in range(N):
        if random.uniform(0, 1) < p0:
            print('    <vehicle id="en_%i" type="type0" route="18_0" depart="%i"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            # lastVeh += 1
            print('    <vehicle id="en_%i" type="type0" route="15_0" depart="%i"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            # lastVeh += 1
            print('    <vehicle id="en_%i" type="type0" route="9_0" depart="%i"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            # lastVeh += 1
            print('    <vehicle id="en_%i" type="type0" route="1_0" depart="%i"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
        if random.uniform(0, 1) < p4:
            print('    <vehicle id="en_%i" type="type4" route="0_4" depart="%i"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            # lastVeh += 1
            print('    <vehicle id="en_%i" type="type4" route="9_4" depart="%i"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            # lastVeh += 1
            print('    <vehicle id="en_%i" type="type4" route="1_4" depart="%i"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
        if random.uniform(0, 1) < p10:
            print('    <vehicle id="en_%i" type="type10" route="0_10" depart="%i"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            # lastVeh += 1
            print('    <vehicle id="en_%i" type="type10" route="4_10" depart="%i"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            # lastVeh += 1
            print('    <vehicle id="en_%i" type="type10" route="15_10" depart="%i"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            # lastVeh += 1
            print('    <vehicle id="en_%i" type="type10" route="1_10" depart="%i"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
        if random.uniform(0, 1) < p18:
            print('    <vehicle id="en_%i" type="type18" route="0_18" depart="%i"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            # lastVeh += 1
            print('    <vehicle id="en_%i" type="type18" route="4_18" depart="%i"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            # lastVeh += 1
            print('    <vehicle id="en_%i" type="type18" route="10_18" depart="%i"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
        if random.uniform(0, 1) < p19:
            print('    <vehicle id="en_%i" type="type19" route="10_19" depart="%i"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            # lastVeh += 1
            print('    <vehicle id="en_%i" type="type19" route="18_19" depart="%i"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            # lastVeh += 1
            print('    <vehicle id="en_%i" type="type19" route="1_19" depart="%i"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            # lastVeh += 1
            print('    <vehicle id="en_%i" type="type19" route="4_19" depart="%i"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
        if random.uniform(0, 1) < p15:
            print('    <vehicle id="en_%i" type="type15" route="10_15" depart="%i"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            # lastVeh += 1
            print('    <vehicle id="en_%i" type="type15" route="18_15" depart="%i"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            # lastVeh += 1
            print('    <vehicle id="en_%i" type="type15" route="19_15" depart="%i"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
        if random.uniform(0, 1) < p9:
            print('    <vehicle id="en_%i" type="type9" route="19_9" depart="%i"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            # lastVeh += 1
            print('    <vehicle id="en_%i" type="type9" route="15_9" depart="%i"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            # lastVeh += 1
            print('    <vehicle id="en_%i" type="type9" route="4_9" depart="%i"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            # lastVeh += 1
            print('    <vehicle id="en_%i" type="type9" route="18_9" depart="%i"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
        if random.uniform(0, 1) < p1:
            print('    <vehicle id="en_%i" type="type1" route="19_1" depart="%i"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            # lastVeh += 1
            print('    <vehicle id="en_%i" type="type1" route="15_1" depart="%i"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            # lastVeh += 1
            print('    <vehicle id="en_%i" type="type1" route="9_1" depart="%i"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
    print("""\n</routes>""", file=routes)
