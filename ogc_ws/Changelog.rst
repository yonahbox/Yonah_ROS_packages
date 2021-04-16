=========================================
Changelog for Ops Ground Control Packages
=========================================

2.1
------------------

- Dynamic calculation of heartbeat timeout for link switch algorithm
- Add a latency simulator to test dynamic calculation of heartbeat timeout
- Bug fix of timestamp not showing in telegram timeout logic (for link switching)
- Bug fix in get_conntype of RuTOS Module
- Bug fixes in GCS Setup scripts (see Yonah_scripts repo)
- Consolidation of AWS Setup folder (see Yonah_scripts repo)

The biggest feature in this release is the addition of a dynamic delay calculator for the link switch heartbeat timeout. It uses TCP's Jacobson Algorithm to dynamically calculate the heartbeat timeout of the link switch algorithm during runtime. A latency simulator is added to simulate high latencies to test the integrity of this feature.

2.0
------------------

Addition of the following:

- Intelligent Link Switching
- Feedback Message System
- File Syncing
- Headers Module

Upgrades:

- Identifiers (transition to a centralised administrator system)
- RQT upgrades to allow dynamic UI, identifiers handling, file syncing
- Simplification of Telegram Link
- Minor changes to SBD Link


1.0
------------------

Initial Release

- Air/Gnd Despatchers
- Air/Gnd Statustext Handlers
- Identifiers Server
- Return-From-Flight (RFF) routine
- RQT GUI
- Telegram, SMS and SBD links