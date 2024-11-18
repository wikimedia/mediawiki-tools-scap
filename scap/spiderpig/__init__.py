from sqlalchemy import create_engine, Engine


def engine(db_filename, connect_args={}) -> Engine:
    return create_engine(
        f"sqlite:///{db_filename}", echo=False, connect_args=connect_args
    )
