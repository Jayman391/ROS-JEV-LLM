from ros2cli.command import CommandExtension, add_subparsers_on_demand


class JevCommand(CommandExtension):
    """jev_ai: status, one-shot decisions, reports, calibration, replay, OPE and search."""

    def add_arguments(self, parser, cli_name):
        self._subparser = parser
        add_subparsers_on_demand(parser, cli_name, '_verb', 'ros2jev.verb', required=False)

    def main(self, *, parser, args):
        if not hasattr(args, '_verb'):
            self._subparser.print_help()
            return 0
        return getattr(args, '_verb').main(args=args)
