"""The experimental manager branches only on eligible competing waiters."""

from math import inf
from importlib import import_module

from examples.RMFS.controlled import create_controlled_model
from examples.RMFS.Tests.run_waiting_lookahead import build,run_model,experiment,RULES


def test_waiting_only_full_horizon_without_nested_lookahead():
    result=experiment('waiting_three')
    row=result['lookahead']
    assert row['completed'] and row['contacts']==0
    assert row['lookahead_calls']>=1
    assert row['rollout_horizon']=='system_completion'
    assert not row['nested_lookahead']
    for decision in row['decisions']:
        assert decision['time']>0
        assert len(decision['candidates'])>=2
        assert set(decision['waiting_since'])==set(decision['candidates'])
        assert all(t<=decision['time'] for t in decision['waiting_since'].values())
        complete=[b for b in decision['branches'] if b['completed']]
        best=min(b['makespan'] for b in complete)
        assert any(b['robot']==decision['selected'] and b['makespan']==best for b in complete)


def test_single_object_never_invokes_lookahead():
    model,_,_=create_controlled_model(
        {'R1':((0,0),(1,0),(2,0),(3,0))},merge_sweeps=True,event_release=True,
        collision_check=False)
    row=run_model(model,lookahead=True,interval=inf)
    assert row['completed'] and row['lookahead_calls']==row['rollouts']==0


def test_production_manager_remains_unchanged():
    from examples.RMFS.TrafficManager import TrafficManager
    model,_,_=create_controlled_model(
        {'R1':((0,0),(1,0))},collision_check=False)
    assert type(model.controller.traffic_manager) is TrafficManager
    assert '?priority' not in model.controller.IP


def test_experiment_is_reproducible_and_does_not_replace_reference_during_restore():
    first_model=build('waiting_three')
    second_model=build('waiting_three')
    a=run_model(first_model,lookahead=True)
    b=run_model(second_model,lookahead=True)
    assert a['completed'] and a['makespan']==b['makespan']
    assert [(d['time'],d['selected']) for d in a['decisions']]==[
        (d['time'],d['selected']) for d in b['decisions']]
    manager=first_model.controller.traffic_manager
    assert manager.viable_plans
    assert all(index<len(manager.plan_predecessors) for index in manager.viable_plans)


def test_each_rollout_uses_its_rule_for_first_and_subsequent_choices():
    result=run_model(build('waiting_three'),lookahead=True,rollout_rules=RULES)
    decision=result['decisions'][0]
    assert {b['rule'] for b in decision['branches']}==set(RULES)
    branches={b['rule']:b for b in decision['branches']}
    assert branches['fifo']['robot']=='R2'
    # At this decision R3 has already advanced one edge, so remaining lengths
    # are tied; all rules correctly use the FIFO tie break.
    assert branches['longest_remaining_distance']['robot']=='R2'
    assert branches['shortest_remaining_distance']['robot']=='R2'
    for branch in decision['branches']:
        assert all(choice['policy']==branch['rule'] for choice in branch['subsequent_choices'])
    assert decision['selected_rule'] in RULES


def test_rule_is_dynamic_snapshot_state():
    from examples.RMFS.Tests.waiting_choice import attach
    model=build('waiting_three')
    manager=attach(model,True).traffic_manager
    state=manager.get_snapshot()
    manager.selection_rule='longest_remaining_distance'
    manager.set_snapshot(state)
    assert manager.selection_rule=='fifo'


def test_distance_rule_uses_current_remaining_path():
    from examples.RMFS.Tests.waiting_choice import attach
    manager=attach(build('waiting_three'),True).traffic_manager
    choices=[next(i for i,(_,name,_) in enumerate(manager.tasks) if name==owner)
             for owner in ('R2','R3')]
    assert manager.tasks[manager.select_by_rule(choices,'longest_remaining_distance')][1]=='R3'
    assert manager.tasks[manager.select_by_rule(choices,'shortest_remaining_distance')][1]=='R2'


def test_rule_contrast_reduces_makespan_and_validates_collision_stopping():
    result=experiment('waiting_three_rule_contrast')
    assert result['baseline']['completed'] and result['lookahead']['completed']
    assert result['improvement_percent']>20
    decision=result['lookahead']['decisions'][0]
    assert {b['robot'] for b in decision['branches']}=={'R2','R3'}
    assert decision['selected_rule']=='longest_remaining_distance'
    assert result['validation']['contacts']==0
    assert result['lrpf']['completed'] and result['lrpf']['lookahead_calls']==0
    assert result['lrpf']['makespan']==result['lookahead']['makespan']
    assert result['lrpf_validation']['completed'] and result['lrpf_validation']['contacts']==0
    assert all(choice['policy']=='longest_remaining_distance' for choice in result['lrpf']['choice_history'])


def test_parent_rule_controls_a_later_contention_without_nested_lookahead():
    result=experiment('waiting_six_two_groups')
    row=result['lookahead']
    assert row['completed'] and row['lookahead_calls']==2 and row['rollouts']==4
    assert result['improvement_percent']>20
    first,second=row['decisions']
    assert second['time']>first['time']
    for branch in first['branches']:
        assert branch['subsequent_choices']  # substantive, not vacuous rule check
        assert all(c['policy']==branch['rule'] and c['time']>first['time']
                   for c in branch['subsequent_choices'])
    assert not row['nested_lookahead'] and not result['validation']['contacts']


def test_mean_completion_uses_final_path_arrivals_and_selects_by_that_objective():
    result=experiment('waiting_three_rule_contrast',objective='mean_completion')
    for key in ('baseline','lrpf','lookahead'):
        row=result[key]
        assert len(row['completion_times'])==3
        assert row['mean_completion']==sum(row['completion_times'].values())/3
        assert max(row['completion_times'].values())==row['makespan']
    row=result['lookahead']
    assert row['objective']=='mean_completion'
    for decision in row['decisions']:
        feasible=[b for b in decision['branches'] if b['completed']]
        chosen=next(b for b in feasible if b['rule']==decision['selected_rule'])
        assert chosen['mean_completion']==min(b['mean_completion'] for b in feasible)
    assert result['validation']['mean_completion']==row['mean_completion']
