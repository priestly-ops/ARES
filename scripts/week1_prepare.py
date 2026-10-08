#!/usr/bin/env python3
"""Derive portable baseline resources from existing ARES assets, without replacing them."""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET
p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--camera',action='store_true');a=p.parse_args()
root=Path(__file__).resolve().parents[1];out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
w=ET.parse(root/'src/ares_simulation/worlds/ares_test_world.sdf');world=w.getroot().find('world')
for actor in world.findall('actor'):world.remove(actor)
inc=world.find('include');world.remove(inc)
m=ET.parse(root/'src/ares_simulation/models/ares_jackal/model.sdf').getroot().find('model');m.find('pose').text=inc.findtext('pose')
if not a.camera:
 link=m.find("link[@name='camera_link']")
 for sensor in link.findall('sensor'):link.remove(sensor)
else:m.find(".//sensor[@type='depth_camera']/always_on").text='true'
world.append(m);ET.indent(w);w.write(out/'world.sdf',encoding='unicode',xml_declaration=True)
