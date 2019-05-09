from __future__ import absolute_import
from __future__ import print_function

# import os
# import sys
# import optparse
# import subprocess
import random

random.seed(42)  # make tests reproducible
N = 2000  # number of time steps, initially 3600
# demand per second from different directions
pE = 1. / 24  # 19
pW = 1. / 25  # 21
pNL = 1. / 20  # 23
pNR = 1. / 30  # 23
pSL = 1. / 30  # 25
pSR = 1. / 20  # 25
with open("data/2sig.rou.xml", "w") as routes:
    print("""<routes>

    <vType id="typeW" accel="0.8" decel="4.5" sigma="0.3" length="5" minGap="2.0" maxSpeed="25" guiShape="passenger"/>
    <vType id="typeNL" accel="0.8" decel="4.5" sigma="0.3" length="5" minGap="2.0" maxSpeed="25" guiShape="passenger"/>
    <vType id="typeSL" accel="0.8" decel="4.5" sigma="0.3" length="5" minGap="2.0" maxSpeed="25" guiShape="passenger"/>
    <vType id="typeNR" accel="0.8" decel="4.5" sigma="0.3" length="5" minGap="2.0" maxSpeed="25" guiShape="passenger"/>
    <vType id="typeSR" accel="0.8" decel="4.5" sigma="0.3" length="5" minGap="2.0" maxSpeed="25" guiShape="passenger"/>
    <vType id="typeE" accel="0.8" decel="4.5" sigma="0.3" length="5" minGap="2.0" maxSpeed="25" guiShape="passenger"/>

    <route id="east-west" edges="ee-e ers rsls lsw w-we" />
    <route id="east-north_left" edges="ee-e ers rsls lsnl" />
    <route id="east-north_right" edges="ee-e ers rsnr" />

    <route id="west-east" edges="we-w wls lsrs rse e-ee" />
    <route id="west-south_left" edges="we-w wls lssl" />
    <route id="west-south_right" edges="we-w wls lsrs rssr" />

    <route id="north_left-south_left" edges="nle-nl nlls lssl sl-sle" />
    <route id="north_left-west" edges="nle-nl nlls lsw w-we" />

    <route id="north_right-south_right" edges="nre-nr nrrs rssr sr-sre" />
    <route id="north_right-west" edges="nre-nr nrrs rsls lsw w-we" />
    <route id="north_right-north_left" edges="nre-nr nrrs rsls lsnl nl-nle" />

    <route id="south_right-north_right" edges="sre-sr srrs rsnr nr-nre" />
    <route id="south_right-east" edges="sre-sr srrs rse e-ee" />

    <route id="south_left-north_left" edges="sle-sl slls lsnl nl-nle" />
    <route id="south_left-south_right" edges="sle-sl slls lsrs rssr sr-sre" />
    <route id="south_left-east" edges="sle-sl slls lsrs rse e-ee" />
    """, file=routes)
    lastVeh = 0
    vehNr = 0
    for i in range(N):
        if random.uniform(0, 1) < pE:
            print('    <vehicle id="en_%i" type="typeE" route="east-north_left" depart="%i" color="#800000"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
            print('    <vehicle id="es_%i" type="typeE" route="east-north_right" depart="%i" color="#F0E68C"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
            print('    <vehicle id="ew_%i" type="typeE" route="east-west" depart="%i" color="#2E8B57"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
        if random.uniform(0, 1) < pW:
            print('    <vehicle id="en_%i" type="typeW" route="west-south_left" depart="%i" color="#42CAFD"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
            print('    <vehicle id="en_%i" type="typeW" route="west-south_right" depart="%i" color="#EC0B43"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
            print('    <vehicle id="en_%i" type="typeW" route="west-east" depart="%i" color="#D3D0CB"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
        if random.uniform(0, 1) < pNL:
            print('    <vehicle id="en_%i" type="typeNL" route="north_left-south_left" depart="%i" color="#42CAFD"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
            print('    <vehicle id="en_%i" type="typeNL" route="north_left-west" depart="%i" color="#2E8B57"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
        if random.uniform(0, 1) < pNR:
            print('    <vehicle id="en_%i" type="typeNR" route="north_right-south_right" depart="%i" color="#EC0B43"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
            print('    <vehicle id="en_%i" type="typeNR" route="north_right-north_left" depart="%i" color="#800000"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
            print('    <vehicle id="en_%i" type="typeNR" route="north_right-west" depart="%i" color="#2E8B57"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
        if random.uniform(0, 1) < pSL:
            print('    <vehicle id="en_%i" type="typeSL" route="south_left-north_left" depart="%i" color="#800000"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
            print('    <vehicle id="en_%i" type="typeSL" route="south_left-south_right" depart="%i" color="#EC0B43"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
            print('    <vehicle id="en_%i" type="typeSL" route="south_left-east" depart="%i" color="#D3D0CB"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
        if random.uniform(0, 1) < pSR:
            print('    <vehicle id="en_%i" type="typeSR" route="south_right-north_right" depart="%i" color="#F0E68C"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
            print('    <vehicle id="en_%i" type="typeSL" route="south_right-east" depart="%i" color="#D3D0CB"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
    print("""\n</routes>""", file=routes)
