Author Javokhir Jambulov
import click
import pytest


@cli.command()
@click.option('--android-driver', default='emulator', help='Specify Android driver type')
def run_tests(android_driver):

    click.echo(f"Running tests with Android driver: {android_driver}")

    # Run pytest programmatically
    result_code = pytest.main(['ros2_android_app_pytest.py'])
    if result_code == 0:
        click.echo("Tests passed!")
    else:
        click.echo("Tests failed!")

if __name__ == '__main__':
    cli()
