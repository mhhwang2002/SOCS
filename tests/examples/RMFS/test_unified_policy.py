"""Expanded lookahead must contain, not replace, the old fixed-LP rollout."""

from itertools import product
from math import isclose

from socs.devs.TESEngin import TESEnginBase
from examples.RMFS.Tests import run_wait_or_go as fixed_lp
from examples.RMFS.Tests import run_unified_policy as expanded


def test_old_fixed_lp_candidate_is_present_with_identical_predicted_makespan():
    old=fixed_lp.run('mixed_four','lookahead_makespan')
    new=expanded.run('mixed_four','lookahead_makespan')
    first=new['decisions'][0]
    assert len(first['branches'])==8
    actual={(b['action'],b['wait_rule'],b['mode']) for b in first['branches']}
    assert actual==set(product((fixed_lp.SEND,fixed_lp.WAIT),('fifo','length'),('fifo','length')))
    previous={b['action']:b for b in old['decisions'][0]['branches']}
    included={b['action']:b for b in first['branches'] if b['fixed_LP_continuation']}
    assert set(included)==set(previous)
    for action in previous:
        for metric in ('makespan','mean_completion','completion_std'):
            assert isclose(included[action][metric],previous[action][metric],rel_tol=0,abs_tol=1e-9)
    assert new['completed'] and new['makespan']<=old['makespan']+1e-9
    assert not new['nested_lookahead']
    assert new['includes_previous_fixed_LP_continuation']
    checked=expanded.validate(new)
    assert checked['completed'] and checked['contacts']==0


def test_wait_rule_and_waiting_priority_are_independent_and_restored():
    model=expanded.build('short_A','lookahead_makespan')
    engine=TESEnginBase(model)
    engine.bVerbose=False
    manager=model.controller.traffic_manager
    assert manager.pending_wait
    snapshot=engine.capture_snapshot()
    manager.set_mode('length',wait_rule='fifo')
    assert manager.wait_action()==fixed_lp.SEND
    owners=('R1','R2')
    choices=[next(i for i,(_,owner,_) in enumerate(manager.tasks) if owner==name) for name in owners]
    assert manager.tasks[manager.select_by_rule(choices)][1]=='R2'
    manager.set_mode('fifo',wait_rule='length')
    assert manager.wait_action()==fixed_lp.WAIT
    assert manager.tasks[manager.select_by_rule(choices)][1]=='R1'
    engine.restore_snapshot(snapshot)
    assert manager.control_mode==manager.wait_rule=='lookahead'
    assert manager.lookahead_enabled


def test_mean_completion_selection_and_random_both_types_are_reproducible():
    row=expanded.run('mixed_four','lookahead_mean')
    assert row['completed']
    for decision in row['decisions']:
        selected=[b for b in decision['branches'] if b['action']==decision['action']
                  and b['mode']==decision['selected_mode'] and b['wait_rule']==decision['selected_wait_rule']][0]
        assert selected['mean_completion']==min(b['mean_completion'] for b in decision['branches'] if b['completed'])
    a=expanded.run('mixed_four','random',seed=3)
    b=expanded.run('mixed_four','random',seed=3)
    assert expanded.signature(a)==expanded.signature(b)
    assert a['makespan']==b['makespan']
