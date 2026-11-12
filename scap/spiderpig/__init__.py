from sqlalchemy import create_engine, Engine


def engine(db_filename, connect_args={}) -> Engine:
    # Several job workers write to the database.  A write transaction waits
    # this long for the write lock before it fails.  The driver default is 5
    # seconds.
    connect_args = {"timeout": 30, **connect_args}

    return create_engine(
        f"sqlite:///{db_filename}", echo=False, connect_args=connect_args
    )
