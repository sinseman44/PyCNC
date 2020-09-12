import logging

#logging.basicConfig(level=logging.CRITICAL,
logging.basicConfig(level=logging.DEBUG,
                    format='[%(levelname)s] %(message)s')


def debug_enable():
    logging.getLogger().setLevel(logging.DEBUG)


def debug_disable():
    logging.getLogger().setLevel(logging.CRITICAL)
