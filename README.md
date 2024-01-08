This template repository is for developers creating new agents for [Upkie](https://github.com/tasts-robots/upkie) wheeled bipeds. All features from the main repository are available in Bazel via `@upkie` targets.

## Setup

- Create a new repository from this template.
- Search for the string "XXX": it indicates template values to adapt, such as the project name to change from ``"upkie_template"`` to your own.
- Adapt the spines to taste, for instance with custom observers.
- Implement your agent in the agent directory.

## Usage

The `Makefile` can be to build and upload your agent to the real robot. There are also some shorthand rules that you may find handy:

```bash
$ make help
Host targets:

    build                    build Raspberry Pi targets
    bullet_spine             start a Bullet simulation spine
    clean                    clean all local build and intermediate files
    upload                   upload built targets to the Raspberry Pi

Raspberry Pi targets:

    run_agent                sandbox agent
    run_mock_spine           run the mock spine on the Raspberry Pi
    run_pi3hat_spine         run the pi3hat spine on the Raspberry Pi
```

Use Bazelisk to run your agent locally:

```bash
$ ./tools/bazelisk run //agent
```
