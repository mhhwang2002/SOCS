"""Connected examples exercise causal interactions, not independent replicas."""

from math import isclose

from examples.RMFS.Tests import run_connected_policy as connected
from examples.RMFS.Tests import run_unified_policy as unified


def test_all_new_cases_are_simple_and_one_connected_interference_group():
    for name,(count,_) in connected.CASES.items():
        model=connected.build(name,'fifo')
        data=model.connected_metadata
        assert data['objects']==count and data['connected'] and data['simple_paths']
        assert len(data['interference_edges'])==count*(count-1)//2
        assert data['reference_candidates']>=2


def test_a_choice_changes_a_different_pair_completion_with_same_rollout_policy():
    row=unified.run('corridor4','lookahead_makespan',model_factory=connected.build)
    first=row['decisions'][0]
    local=row['execution_trace']['wait_actions'][0]
    by_action={b['action_name']:b for b in first['branches']
               if b['mode']=='length' and b['wait_rule']=='fifo'}
    a,b=by_action['send_now'],by_action['wait_for_B']
    others=set(row['completion_times'])-{local['a'],local['b']}
    assert any(not isclose(a['completion_times'][name],b['completion_times'][name],abs_tol=1e-9)
               for name in others)
    assert row['completed'] and not row['nested_lookahead']


def test_six_object_lookahead_improvement_and_collision_stopping():
    fifo=unified.run('corridor6','fifo',model_factory=connected.build)
    chosen=unified.run('corridor6','lookahead_makespan',model_factory=connected.build)
    assert fifo['completed'] and chosen['completed']
    assert chosen['makespan']<fifo['makespan']
    assert chosen['lookahead_calls']>0
    validation=unified.validate(chosen,model_factory=connected.build)
    assert validation['completed'] and validation['contacts']==0


def test_reversed_tail_fixture_is_retained_even_without_improvement():
    fifo=unified.run('corridor4_reverse_tails','fifo',model_factory=connected.build)
    chosen=unified.run('corridor4_reverse_tails','lookahead_makespan',model_factory=connected.build)
    assert fifo['completed'] and chosen['completed']
    assert chosen['makespan']==fifo['makespan']
