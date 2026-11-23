from examples.RMFS.TrafficManager import TrafficManager
from examples.RMFS.RoadMap import RoadMap
from examples.RMFS.path_geometry import Path,PathPosition,Subpath,TuplePath


class DeferredManager(TrafficManager):
    armed=False
    def reconsider(self,time):
        if self.armed:
            super().reconsider(time)


def manager(crossing=False):
    a=Path('R1',((0.,0.),(4.,0.)))
    b=Path('R2',((2.,-3.),(2.,3.))) if crossing else Path('R2',((0.,5.),(2.,5.)))
    parts=(Subpath(a,PathPosition(0),PathPosition(1)),Subpath(b,PathPosition(0),PathPosition(1)))
    plan=TuplePath(('R1','R2'),((parts[0],b.point(0)),(a.point(1),parts[1])))
    return DeferredManager(RoadMap({'R1':a,'R2':b}),plan,event_release=True)


def test_fifo_selects_earlier_waiter_not_reference_list_order():
    m=manager()
    m.task_wait_since={0:5.,1:2.}
    snapshot=m.get_snapshot()
    m.armed=True;m.reconsider(10.)
    assert [c['robot'] for c in m.command_buffer]==['R2','R1']
    assert not m.task_wait_since
    m.set_snapshot(snapshot)
    assert m.task_wait_since=={0:5.,1:2.}


def test_fifo_does_not_override_interference_precedence():
    m=manager(crossing=True)
    m.task_wait_since={0:5.,1:2.}
    m.armed=True;m.reconsider(10.)
    assert [c['robot'] for c in m.command_buffer]==['R1']
    assert m.task_wait_since[1]==2.


