import click

@click.group()
def cli():
    pass

@cli.command()
@click.option('--start', is_flag=True, help='Start error detection')
def error_detection(start):
    if start:
        click.echo("Error detection started.")

@cli.command()
@click.option('--stop', is_flag=True, help='Stop error detection')
def stop_error_detection(stop):
    if stop:
        click.echo("Error detection stopped.")

if __name__ == '__main__':
    cli()

