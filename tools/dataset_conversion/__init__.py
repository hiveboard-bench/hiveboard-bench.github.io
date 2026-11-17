"""Convert datasets recorded outside HiveBoard into ``datahive_episode_v1``.

The package writes through the ``datahive`` package's own API so that the
output is accepted by ``datahive validate``, ``datahive check`` and
``datahive upload`` without modification.
"""

__version__ = "0.1.0"
