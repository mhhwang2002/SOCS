"""Capture actual Panda3D initial scenes for the research paper (no UI loop)."""
import argparse
from importlib import import_module
import json
from pathlib import Path
import sys

from panda3d.core import Filename,loadPrcFileData
from direct.showbase.ShowBase import ShowBase
from .run_paper_validation import fixture_factory,saved_factory


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case')
    parser.add_argument('--data',type=Path)
    parser.add_argument('--sample',type=int,default=1)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if bool(args.case)==bool(args.data):parser.error('Choose either --case or --data.')
    loadPrcFileData('', 'window-type offscreen')
    loadPrcFileData('', 'audio-library-name null')
    if args.case:
        source=import_module('examples.RMFS.Tests.'+args.case)
        routes=source.PATHS;grid=getattr(source,'GRID',None)
        builder=fixture_factory(args.case)
        title=args.case
    else:
        from ..GridPath import make_grid
        data=json.loads(args.data.read_text())
        sample=next(s for s in data['samples'] if s['index']==args.sample-1)
        routes={n:tuple(map(tuple,points)) for n,points in sample['routes'].items()}
        grid=make_grid(data['config']['grid_size'],data['config']['spacing_m'])
        builder,_=saved_factory(data,args.sample-1)
        title=f'{args.data.stem}, case {args.sample}'
    args.output.parent.mkdir(parents=True,exist_ok=True)
    def capture(viewer):
        viewer.camera_view('iso')
        viewer.draw(0)
        assert viewer.engine.tNow==0 and all(r.phase=='Wait' for r in viewer.robots)
        for node in viewer.render.findAllMatches('**/+TextNode'):
            name=node.getName()
            if name.startswith('(') or (len(routes)>8 and (name.startswith('motion-') or name.startswith('R'))):
                node.hide()
        viewer.status.setText(f'{len(routes)} mobile robots | Initial prescribed paths and sweeps\n'
                              'Green Frogeye bodies | Path colours identify objects')
        viewer.status.setScale(.035)
        for _ in range(5):viewer.graphicsEngine.renderFrame()
        assert viewer.win.getScreenshot().write(Filename.fromOsSpecific(str(args.output.resolve())))
        viewer.destroy()
    ShowBase.run=capture
    sys.argv=[title,'--no-rack','--white-background','--no-collision-check']
    import_module('examples.RMFS.Tests.01_01_RobotPath_Straight').main(
        name=title,no_rack=True,grid=grid,robot_paths=routes,
        model_factory=lambda **options:builder('fifo',False))
    print(args.output,flush=True)


if __name__=='__main__':main()
