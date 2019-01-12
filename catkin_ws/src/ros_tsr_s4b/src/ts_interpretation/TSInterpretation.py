import xml.etree.ElementTree as ET


# classes
class TSRuleBase():
    def __init__(self, xml_rules):
        self.ts_factory = TSRuleComponentFactory()
        self._load_rules(xml_rules)

    def _load_rules(self, selfxml_rules):
        tree = ET.parse(selfxml_rules)
        xml_root = tree.getroot()
        self.ts_factory.createComponentByName(xml_root.tag, None, False)
        parent = self.ts_factory.getComponentByName(xml_root.tag)
        self._loadElements(xml_root, parent)
        self._setInvalidatesRelations(xml_root)

    def _loadElements(self, xml_elem, parent):
        name = xml_elem.get('name')
        tag = xml_elem.tag
        if name is not None:
            is_leaf = (tag == "ts")
            self.ts_factory.createComponentByName(name, parent=parent, is_leaf=is_leaf)
            parent = self.ts_factory.getComponentByName(name)
        xml_groups = xml_elem.findall("ts_group")
        for xml_group in xml_groups:
            self._loadElements(xml_group, parent)
        leafs = xml_elem.findall("ts")
        for leaf in leafs:
            self._loadElements(leaf, parent)

    def _setInvalidatesRelations(self, xml_elem):
        xml_groups = xml_elem.findall("ts_group")
        for xml_group in xml_groups:
            self._setInvalidatesRelations(xml_group)
        leafs = xml_elem.findall("ts")
        for leaf in leafs:
            self._setInvalidatesRelations(leaf)

        name = xml_elem.get('name')

        if name is not None:
            ts_elem = self.ts_factory.getComponentByName(name)
            xml_invalidates = xml_elem.findall('invalidates')
            for xml_invalidate in xml_invalidates:
                inv_name = xml_invalidate.get('name')
                inv_elem = self.ts_factory.getComponentByName(inv_name)

                ts_elem.addInavlidatesRelation(inv_elem)

    def update(self, new_sign_name, old_sign_names):
        assert str(new_sign_name)
        new_sign = self.ts_factory.getComponentByName(new_sign_name)
        assert isinstance(new_sign, TSRuleComponent)
        updated_names = [];
        updated_names.append(new_sign_name)
        for old_sign_name in old_sign_names:
            old_sign = self.ts_factory.getComponentByName(old_sign_name)
            assert isinstance(old_sign, TSRuleComponent)
            if not new_sign.invalidates(old_sign) and not (new_sign_name == old_sign_name):
                updated_names.append(old_sign_name)
        return updated_names
    def getByName(self,name):
        return self.ts_factory.getComponentByName(name=name)

class TSRuleComponentFactory():
    def __init__(self):
        self.instances = {}

    def createComponentByName(self, name, parent=None, is_leaf=False):
        if name not in self.instances.keys():
            if is_leaf:
                self.instances[name] = TSRuleLeaf(name, parent)
            else:
                self.instances[name] = TSRuleComposite(name, parent)

    def getComponentByName(self, name):
        return self.instances[name]


class TSRuleComponent(object):
    def __init__(self, name, parent=None):
        self.name = name
        self.parent = parent
        self.invalidates_list = []
    def getParent(self):
        return self.parent
    def addInavlidatesRelation(self, tsComponent):
        assert isinstance(tsComponent, TSRuleComponent)
        self.invalidates_list.append(tsComponent)

    def getInvalidatesList(self):
        if self.parent is None:
            return self.invalidates_list
        return self.parent.invalidates_list + self.invalidates_list

    def invalidates(self, tsComponent):
        names_to_delete = [c.name for c in self.getInvalidatesList()]
        names = tsComponent.getNames()
        for name_to_delete in names_to_delete:
            if name_to_delete in names:
                return True
        return False

    def getNames(self):
        parents_names = []
        if not self.parent is None:
            parents_names = self.parent.getNames()
        return [self.name] + parents_names
    def isAn(self,name):
        return name in self.getNames()
    def __str__(self):
        invalidates_str = ""
        for invalidates in self.getInvalidatesList():
            invalidates_str = invalidates_str + "," + invalidates.name
        parent_name = "Root"
        if self.parent is not None:
            parent_name = self.parent.name
        return "{},parent:{},invalidates:{}".format(self.name, parent_name, invalidates_str)


class TSRuleComposite(TSRuleComponent):
    def __init__(self, name, parent=None):
        assert ((parent is None or isinstance(parent, TSRuleComponent)) and not isinstance(parent, TSRuleLeaf))
        super(TSRuleComposite, self).__init__(name, parent)
        self.children = []

    def addChild(self, child):
        assert isinstance(child, TSRuleLeaf)
        self.children.append(child)


class TSRuleLeaf(TSRuleComponent):
    def __init__(self, name, parent):
        assert isinstance(parent, TSRuleComposite)
        super(TSRuleLeaf, self).__init__(name, parent)


