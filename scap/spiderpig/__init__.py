from sqlalchemy import create_engine, Engine


def engine(db_filename) -> Engine:
    return create_engine(f"sqlite:///{db_filename}", echo=False)
