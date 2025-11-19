These are identical python tests to those found in the gtsam repository to ensure compatibility. The only minor changes are as follows:
- OrderingType is no longer nested under Ordering, this was changed in `test_DiscreteFactorGraph.py`.
- Serialization tests have been removed since serialization is not currently supported in gtsam-nb.
- 