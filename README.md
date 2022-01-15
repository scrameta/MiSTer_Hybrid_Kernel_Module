
# minimigirq
Expose minimig irq to linux, so I can wait on an ioctl for it.
Allow memory mapping with various caching options, since the fast ram is outside the ram that linux knows about. So mapping it with /dev/mem is uncached.

