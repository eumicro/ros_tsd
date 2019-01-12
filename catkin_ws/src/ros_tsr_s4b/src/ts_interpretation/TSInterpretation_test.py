import unittest
from ros_tsr_s4b.src.ts_interpretation.TSInterpretation import *

class TSRuleBaseTest(unittest.TestCase):

    def setUp(self):
        self.rules = TSRuleBase("/home/eumicro/catkin_ws/src/ros_tsr_s4b/models/interpretation/ts_rules.xml")
        self.assertIsInstance(self.rules,TSRuleBase)

    def test_single_initalization(self):
        sign_name1 = "Z101"
        sign_name2 = "traffic_sign"
        sign_name3 = "speed-limit"
        sign_name4 = "Z274-20"
        ts1 = self.rules.ts_factory.getComponentByName(sign_name1)
        ts2 = self.rules.ts_factory.getComponentByName(sign_name2)
        ts3 = self.rules.ts_factory.getComponentByName(sign_name3)
        ts4 = self.rules.ts_factory.getComponentByName(sign_name4)
        self.assertIsInstance(ts1,TSRuleLeaf)
        self.assertIsInstance(ts2, TSRuleComposite)
        self.assertIsInstance(ts3, TSRuleComposite)
        self.assertIsInstance(ts4, TSRuleLeaf)
    def test_single_invalidates(self):
        sign_name1 = "Z206"
        sign_name2 = "traffic_sign"
        sign_name3 = "speed-limit"
        sign_name4 = "Z274-20"

        old_signs = [sign_name2,sign_name3,sign_name4]
        new_signs=self.rules.update(sign_name1,old_signs)
        self.assertTrue(len(new_signs)==1)
        self.assertEqual(new_signs[0],sign_name1)

if __name__ == '__main__':
    unittest.main()
