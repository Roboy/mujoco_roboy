import xml.etree.ElementTree as ET
import os.path

defaultsitecolor = '1 0.7 0 0.2'
defaulttendoncolor = '0.1 0.4 0.6 1'
torque_d = 0.023
gear_d = 53
winchdiameter_d = 0.008

#####
# User inputs for model creation

while True:
    modelfile = input('Name of model (Default: model.xml -> Enter) ')
    if modelfile == '':
        modelfile = 'model.xml'
    if os.path.isfile(modelfile):
        break
    else:
        print('%s not found' % modelfile)

while True:
    cardsflowfile = input(
        'Name of cardsflow (Default: cardsflow.xml -> Enter) ')
    if cardsflowfile == '':
        cardsflowfile = 'cardsflow.xml'
    if os.path.isfile(modelfile):
        break
    else:
        print('%s not found' % cardsflowfile)

outputfile = input('Name of output (Default: model_fused.xml -> Enter) ')
if outputfile == '':
    outputfile = 'model_fused.xml'


sitecolor = input('Color for Sites. (Default: %s) ' % defaultsitecolor)
if sitecolor == '':
    sitecolor = defaultsitecolor

tendoncolor = input('Color for Tendons. (Default: %s) ' % defaulttendoncolor)
if tendoncolor == '':
    tendoncolor = defaulttendoncolor


torque = input('Motor Torque. (Default: %s) ' % torque_d)
if torque == '':
    torque = torque_d
else:
    torque = float(torque)
    
gear = input('Gear Ratio. (Default: %s) ' % gear_d)
if gear == '':
    gear = gear_d
else:
    gear = int(gear)
    
winchdiameter = input('Winch Diameter. (Default: %s) ' % winchdiameter_d)
if winchdiameter == '':
    winchdiameter = winchdiameter_d
else:
    winchdiameter = float(winchdiameter)
    
tendonforce = str(round((torque*gear)/winchdiameter,2))
print('Motors will generate a force of %sN on the tendons' % tendonforce)
    

modeltree = ET.parse(modelfile)
cardstree = ET.parse(cardsflowfile)

modelroot = modeltree.getroot()
cardsroot = cardstree.getroot()

# add site and tendon default color
default = modeltree.find('default')
if not default:
    default = ET.Element('default')
    modelroot.append(default)
defaultsite = default.find('site')
if defaultsite == None:
    defaultsite = ET.Element('site')
    default.append(defaultsite)
defaultsite.set('rgba', sitecolor)
defaulttendon = default.find('tendon')
if defaulttendon == None:
    defaulttendon = ET.Element('tendon')
    default.append(defaulttendon)
defaulttendon.set('rgba', tendoncolor)

defaultmotor = default.find('motor')
if defaultmotor == None:
    defaultmotor = ET.Element('motor')
    default.append(defaultmotor)
defaultmotor.set('gear', '-1')
defaultmotor.set('forcelimited', 'true')
defaultmotor.set('forcerange', "0 "+tendonforce)
defaultmotor.set('ctrllimited', 'true')
defaultmotor.set('ctrlrange', "0 "+tendonforce)



####
# actual model creation

# iter (findall) over all myoMuscles of cardstree
for myoMuscle in cardstree.iter('myoMuscle'):
    # name of Muscle from cardsflow for tendon and actuator
    muscleName = myoMuscle.get('name')
    # tendon location model
    tendon = modeltree.find('tendon')
    # actuator location model
    actuator = modeltree.find('actuator')
    # new spatial for each myoMuscle
    spatial = ET.Element('spatial')
    spatial.set('name', muscleName)
    tendon.append(spatial)
    # new muscle for each myoMuscle
    muscle = ET.Element('motor')
    muscle.set('name', muscleName)
    muscle.set('tendon', muscleName)
    actuator.append(muscle)

    # find all links for each muscle
    for link in myoMuscle.findall('link'):
        linkname = link.get('name')
        # find the body of modeltree which has the link
        body = modelroot.find('.//body[@name="%s"]' % linkname)
        # some bodys do not exist, e.g. "torso"
        if body:
            # correct Site position is geomPosition + viaPointPosition
            # geomPosition = [*map(float, geomPosition)] changes text list in float list
            geomPosition = [*map(float, body.find('geom').get('pos').split())]
            # go through all viaPoints of the link and enumerate
            for count, viaPoint in enumerate(link.getchildren(), start=1):
                viaPointPosition = [*map(float, viaPoint.text.split())]
                # if more than one viaPoint, the sitenames are enumerated
                if len(link.getchildren()) > 1:
                    sitename = myoMuscle.get(
                        'name')+'_'+link.get('name')+'_'+str(count)
                else:
                    sitename = myoMuscle.get('name')+'_'+link.get('name')

                site = ET.Element('site')
                sitePosition = [a + b for a,
                                b in zip(geomPosition, viaPointPosition)]
                sitePositionString = ' '.join(str(e) for e in sitePosition)
                site.set('name', sitename)
                site.set('pos', sitePositionString)
                body.append(site)

                site_tendon = ET.Element('site')
                site_tendon.set('site', sitename)
                spatial.append(site_tendon)

        else:
            print('ERROR\nBody of cardsflow.xml not found in model.xml!')

modeltree.write(outputfile)
