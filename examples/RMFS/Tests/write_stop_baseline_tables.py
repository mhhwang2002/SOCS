"""Generate the paper's fresh stop-only tables directly from measured JSON."""
from argparse import ArgumentParser
from collections import OrderedDict
from json import loads,dumps
from pathlib import Path


def main():
    parser = ArgumentParser(description=__doc__)
    root = Path(__file__).parent/'results'
    parser.add_argument('--input',type=Path,default=root/'stop_only_baseline_20261121.json')
    parser.add_argument('--output',type=Path,default=root/'stop_only_baseline_tables_20261121.tex')
    args = parser.parse_args()
    data = loads(args.input.read_text(encoding='utf-8'))
    assert data['status']=='complete'
    grouped = OrderedDict()
    for case in data['results']:
        assert 'on' not in case and case['validated'] and case['off']['extension_commands']==0
        grouped.setdefault(case['case'],{})[case['policy']] = case
    lines = ['% Generated from the post-fix stop-only measurements. Do not hand-edit numbers.']
    def table(caption,label,columns,head,rows):
        lines.extend([r'\begin{table}[htbp]',r'\centering\small',r'\setlength{\tabcolsep}{4pt}',
            '\\caption{'+caption+'}',r'\label{'+label+'}',r'\begin{tabular}{'+columns+'}',
            r'\toprule',head+r'\\',r'\midrule',*rows,r'\bottomrule',r'\end{tabular}',r'\end{table}',''])
    makespan,completion,runtime = [],[],[]
    summary = dict(route_sets=len(grouped),policy_cases=len(data['results']),
        timed_executions=len(data['results'])*data['repeats'],collision_replays=len(data['results']),
        extension_commands=0,contacts=0,lp_makespan_better=0,lp_makespan_worse=0,lp_makespan_tied=0,
        lp_mean_better=0,lp_mean_worse=0,lp_mean_tied=0,lp_holds=0,lp_arbitrations=0)
    for row,(name,policies) in enumerate(grouped.items(),1):
        a,b = policies['fifo'],policies['lp']
        assert a['off']['input_sha256']==b['off']['input_sha256']
        f,l = a['off'],b['off']
        if name.startswith('dag_'):
            title = f"D{a['robots']}-{a['seed']%100}"
        elif name.startswith('interior_'):
            title = 'Interior'
        else:
            title = 'Approach A' if '03_08' in name else 'Braking B'
        geometry = a['geometry']
        grid = a['grid'].replace('x',r'\times')
        makespan.append(f"{row} & {title} & ${grid}$ & {a['robots']} & "
            f"{geometry['directed_pairwise_interval_count']} & {geometry['total_pairwise_interval_m']:.1f} & "
            f"{f['makespan_s']:.3f} & {l['makespan_s']:.3f}"+r'\\')
        completion.append(f"{row} & {f['mean_completion_s']:.3f} & {f['completion_std_s']:.3f} & "
            f"{l['mean_completion_s']:.3f} & {l['completion_std_s']:.3f}"+r'\\')
        runtime.append(f"{row} & {1000*f['median_wall_s']:.2f} & {1000*l['median_wall_s']:.2f} & "
            f"{f['intermediate_stops']} & {l['intermediate_stops']} & {l['optional_holds']} & "
            f"{l['arbitration_decisions']}"+r'\\')
        for metric,key in (('makespan_s','makespan'),('mean_completion_s','mean')):
            delta = l[metric]-f[metric]
            summary['lp_'+key+('_tied' if abs(delta)<1e-7 else '_better' if delta<0 else '_worse')] += 1
        summary['lp_holds'] += l['optional_holds']
        summary['lp_arbitrations'] += l['arbitration_decisions']
    table('Post-fix stop-only baseline: geometry and makespan in seconds. Both policies use safe-prefix approach and sweep merging; motion extension is OFF.',
          'tab:stop-baseline-makespan','rlcrrrrr',
          r'ID & Case & Grid & $n$ & $K_I$ & $L_I$ (m) & FIFO $C_{\max}$ & LP $C_{\max}$',makespan)
    table('Post-fix stop-only baseline: mean and population standard deviation of final path-completion times, in seconds. IDs match Table~\\ref{tab:stop-baseline-makespan}.',
          'tab:stop-baseline-mean','rrrrr',r'ID & FIFO $\overline C$ & FIFO $\sigma_C$ & LP $\overline C$ & LP $\sigma_C$',completion)
    table('Stop-only execution cost and decisions. Wall time is the median of three DEVS-only runs, excluding model/certificate construction. Stops exclude final arrivals; LP holds and waiter arbitrations are counted per execution.',
          'tab:stop-baseline-runtime','rrrrrrr',r'ID & FIFO (ms) & LP (ms) & FIFO stops & LP stops & LP holds & LP selections',runtime)
    args.output.write_text('\n'.join(lines)+'\n',encoding='utf-8')
    args.output.with_suffix('.summary.json').write_text(dumps(summary,indent=2)+'\n',encoding='utf-8')
    print(dumps(summary,indent=2))


if __name__ == '__main__':
    main()
