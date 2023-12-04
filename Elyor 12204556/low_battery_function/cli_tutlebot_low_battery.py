#!/usr/bin/env python

import click
import os

@click.group()
def cli():
    pass

@click.command()
@click.option('--rate', default=1, help='Publish rate in Hz')
def publisher(rate):
    os.system(f'./publisher_node.py --rate {rate}')

@click.command()
def subscriber():
    os.system('./subscriber_node.py')

cli.add_command(publisher)
cli.add_command(subscriber)

if __name__ == '__main__':
    cli()

