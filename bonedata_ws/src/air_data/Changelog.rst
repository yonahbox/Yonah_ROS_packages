=============================
Changelog for package air_data
=============================

2.0
------------------

- Redefine Data Link as Data (Tech) link 2.0; a separate cellular data link (based on telegram) will be created in the Ops Ground Control project
- Allow for multiple aircraft-GCS pairs to communicate over a single server
- Remove the reliance on AWS Private Key (use public key authentication instead)
- Upgrade **all** scripts to Python 3
- Compatibility with Ubuntu 20.04
- Breakout AWS Server hostname and IP into a ros param (to allow easier modification if needed)

1.0
------------------

- Initial Release
- Send MAVlink data from one aircraft to one GCS
- Two-way diagnostic information