from unittest import TestCase
import SIPP

class TestSIPP(TestCase):
    def test_getEightConnectedNeighbors(self):
        print(SIPP.SIPP.getEightConnectedNeighbors([1,2,3], 0.1, 0.1, 0.1))
        self.fail()
