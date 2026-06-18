from src.cli import build_parser


def test_formal_cli_exposes_snapshot_replay():
    args = build_parser().parse_args(
        ["snapshot-replay", "--snapshot-manifest", "configs/camera_snapshot.example.yaml"]
    )

    assert args.command == "snapshot-replay"
    assert args.snapshot_manifest == "configs/camera_snapshot.example.yaml"


def test_formal_cli_does_not_expose_legacy_image_commands():
    parser = build_parser()
    subcommands = parser._subparsers._group_actions[0].choices

    assert "demo" not in subcommands
    assert "prepare-images" not in subcommands
