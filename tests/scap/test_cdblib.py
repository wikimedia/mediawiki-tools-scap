import io

from scap import cdblib


def test_writer_put():
    fp = io.BytesIO()
    writer = cdblib.Writer(fp)
    writer.put("mediawiki", "\U0001F33B")
    writer.finalize()

    fp.seek(0)

    reader = cdblib.Reader(fp.read())

    assert reader.items() == [("mediawiki", "\U0001F33B")]
