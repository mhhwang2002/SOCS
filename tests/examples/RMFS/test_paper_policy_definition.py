from types import SimpleNamespace
from examples.RMFS.path_geometry import Path,PathPosition,Subpath
from examples.RMFS.Tests.run_generated_policy import ResidualPolicyManager


def test_lp_compares_at_safe_waiting_positions_and_does_not_wait_on_tie():
    manager=ResidualPolicyManager.__new__(ResidualPolicyManager)
    manager.wait_rule='length'
    manager.roadmap=SimpleNamespace(paths={'B':Path('B',((0.,0.),(4.,0.),(8.,0.),(12.,0.)))})
    manager.remaining_length=lambda name:8.
    manager.pending_wait={'a':'A','b':'B','gb':PathPosition(1)}
    assert manager.wait_action()=='send_now'
    manager.pending_wait['gb']=PathPosition(0)
    assert manager.wait_action()=='wait_for_B'
    manager.pending_wait['gb']=PathPosition(2)
    assert manager.wait_action()=='send_now'


def test_fifo_always_sends_without_intentional_wait():
    manager=ResidualPolicyManager.__new__(ResidualPolicyManager)
    manager.wait_rule='fifo'
    manager.pending_wait={'a':'A','b':'B'}
    assert manager.wait_action()=='send_now'


def test_release_queue_fifo_and_lp_choose_by_their_declared_rule():
    manager=ResidualPolicyManager.__new__(ResidualPolicyManager)
    paths={'A':Path('A',((0.,0.),(4.,0.))),'B':Path('B',((0.,4.),(8.,4.)))}
    manager.roadmap=SimpleNamespace(remaining_subpath=lambda n:Subpath(paths[n],PathPosition(0),PathPosition(1)))
    manager.tasks=((0,'A',None),(0,'B',None))
    manager.wait_since={'A':0.,'B':1.}
    manager.control_mode='fifo'
    assert manager.select_by_rule((0,1))==0
    manager.control_mode='length'
    assert manager.select_by_rule((0,1))==1
    paths['B']=Path('B',((0.,4.),(4.,4.)))
    assert manager.select_by_rule((0,1))==0
