import importlib.util

# T374983: Initialize SQLAlchemy only when the package is available (i.e. when on a deployment server)
if importlib.util.find_spec("sqlalchemy") is not None:
    from sqlalchemy import create_engine, Engine

    def engine(db_filename) -> Engine:
        return create_engine(f"sqlite:///{db_filename}", echo=False)
