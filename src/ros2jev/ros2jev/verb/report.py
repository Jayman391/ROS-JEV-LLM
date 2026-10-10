"""ros2 jev report: decision-record metrics as a self-contained HTML page (A5.5)."""
from ros2cli.verb import VerbExtension


class ReportVerb(VerbExtension):
    """Write metrics + charts for a decisions JSONL file (from decision_logger)."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument('records', nargs='?', help='decisions JSONL (omit with --demo)')
        parser.add_argument('--labels', help='labels JSONL (enables calibration charts)')
        parser.add_argument('-o', '--output', default='jev_report.html')
        parser.add_argument('--demo', action='store_true', help='use synthetic sample records')

    def main(self, *, args):
        from jev_ai.viz import report
        argv = ([args.records] if args.records else []) + ['-o', args.output] + \
            (['--labels', args.labels] if args.labels else []) + (['--demo'] if args.demo else [])
        return report.main(argv)
