from jinja2 import Template

from autonetkit.design.utils import filters
from autonetkit.design.utils.filters import find_node_by_label
from autonetkit.design.utils.general import group_by
from autonetkit.design.utils.graph_utils import topology_to_nx_graph, wrap_node_ids
from autonetkit.network_model.network_model import NetworkModel
from autonetkit.network_model.types import DeviceType, PortType
from autonetkit.webserver.publish import publish_model_to_webserver

network_model = NetworkModel()

t_phy = network_model.create_topology("physical")

# r0 = t_phy.create_node(DeviceType.ROUTER, "r0")

# r1 = t_phy.create_node(DeviceType.ROUTER, "r1")
# r2 = t_phy.create_node(DeviceType.ROUTER, "r2")
# r3 = t_phy.create_node(DeviceType.ROUTER, "r3")
# r4 = t_phy.create_node(DeviceType.ROUTER, "r4")
r = {}

for num in range(0, 49):
    r[num] = t_phy.create_node(DeviceType.ROUTER, "r" + str(num))
# r5 = t_phy.create_node(DeviceType.ROUTER, "r5")
# h1 = t_phy.create_node(DeviceType.HOST, "h1")
# h2 = t_phy.create_node(DeviceType.HOST, "h2")
dRouter_id = {}

# r0.set("x", 0)
# r0.set("y", 0)
# r0.set("asn", 1)

properties = {
    "r0": (250, 0, 1),
    "r1": (0, 250, 1),
    "r2": (250, 250, 1),
    "r3": (250, 0, 1),
    "r4": (0, 250, 1),
    "r5": (250, 250, 1),
    "r6": (250, 0, 1),
    "r7": (0, 250, 1),
    "r8": (250, 250, 1),
    "r9": (250, 250, 1),
    "r10": (250, 0, 1),
    "r11": (0, 250, 1),
    "r12": (250, 250, 1),
    "r13": (250, 0, 1),
    "r14": (0, 250, 1),
    "r15": (250, 250, 1),
    "r16": (250, 0, 1),
    "r17": (0, 250, 1),
    "r18": (250, 250, 1),
    "r19": (250, 0, 1),
    "r20": (0, 250, 1),
    "r21": (250, 250, 1),
    "r22": (250, 0, 1),
    "r23": (0, 250, 1),
    "r24": (250, 250, 1),
    "r25": (250, 0, 1),
    "r26": (0, 250, 1),
    "r27": (250, 250, 1),
    "r28": (250, 0, 1),
    "r29": (0, 250, 1),
    "r30": (250, 250, 1),
    "r31": (250, 250, 1),
    "r32": (250, 0, 1),
    "r33": (0, 250, 1),
    "r34": (250, 250, 1),
    "r35": (250, 0, 1),
    "r36": (0, 250, 1),
    "r37": (250, 250, 1),
    "r38": (250, 0, 1),
    "r39": (0, 250, 1),
    "r40": (250, 250, 1),
    "r41": (250, 0, 1),
    "r42": (0, 250, 1),
    "r43": (250, 250, 1),
    "r44": (250, 250, 1),
    "r45": (250, 250, 1),
    "r46": (250, 250, 1),
    "r47": (250, 250, 1),
    "r48": (250, 250, 1)
}

for node_id, (x, y, asn) in properties.items():
    node = find_node_by_label(t_phy, node_id)
    node.set("x", x)
    node.set("y", y)
    node.set("asn", asn)


# create ports
# r1p1 = r1.create_port(PortType.PHYSICAL)
# # h1p1 = h1.create_port(PortType.PHYSICAL)
# # and link them
# # t_phy.create_link(r1p1, h1p1)
# t_phy.create_link(r1p1)

# or create directly
# t_phy.create_link(r1.create_port(PortType.PHYSICAL), r2.create_port(PortType.PHYSICAL))
# or in a loop
pairs = [(r[0], r[1]), (r[0], r[3]), (r[0], r[4]),
         (r[2], r[39]), (r[3], r[39]),
         (r[4], r[39]),(r[4], r[5]),
         (r[4], r[6]), 
         (r[4], r[45]),
         (r[4], r[13]),
         (r[4], r[21]),
         (r[4], r[22]),
         (r[4], r[25]),
         (r[5], r[8]),
         (r[6], r[9]),
         (r[6], r[7]),
         (r[7], r[20]),
         (r[8], r[9]),
         (r[9], r[33]),
         (r[9], r[34]),
         (r[9], r[10]),
         (r[9], r[11]),
         (r[9], r[13]),
         (r[9], r[14]),
         (r[9], r[45]),
         (r[9], r[23]),
         (r[9], r[25]),
         (r[9], r[27]),
         (r[9], r[31]),
         (r[12], r[13]),
         (r[12], r[14]),
         (r[13], r[33]),
         (r[13], r[43]),
         (r[13], r[45]),
         (r[13], r[14]),
         (r[13], r[25]),
         (r[13], r[27]),
         (r[13], r[31]),
         (r[14], r[37]),
         (r[14], r[15]),
         (r[14], r[18]),
         (r[15], r[17]),
         (r[15], r[18]),
         (r[16], r[33]),
         (r[16], r[17]),
         (r[17], r[18]),
         (r[18], r[19]),
         (r[18], r[33]),
         (r[20], r[21]),
         (r[20], r[39]),
         (r[21], r[24]),
         (r[21], r[23]),
         (r[21], r[48]),
         (r[24], r[25]),
         (r[25], r[33]),
         (r[25], r[43]),
         (r[25], r[45]),
         (r[25], r[48]),
         (r[25], r[26]),
         (r[25], r[27]),
         (r[26], r[28]),
         (r[27], r[43]),
         (r[27], r[45]),
         (r[27], r[47]),
         (r[27], r[28]),
         (r[29], r[41]),
         (r[30], r[43]),
         (r[31], r[32]),
         (r[31], r[36]),
         (r[31], r[38]),
         (r[31], r[45]),
         (r[32], r[37]),
         (r[32], r[47]),
         (r[33], r[34]),
         (r[35], r[45]),
         (r[37], r[38]),
         (r[39], r[40]),
         (r[39], r[41]),
         (r[41], r[42]),
         (r[42], r[43]),
         (r[43], r[44]),
         (r[43], r[45]),
         (r[44], r[45]),
         (r[45], r[46]),
         (r[46], r[47])]
for n1, n2 in pairs:
    t_phy.create_link(n1.create_port(PortType.PHYSICAL), n2.create_port(PortType.PHYSICAL))

# create loopbacks
routers = filters.routers(t_phy)
for node in t_phy.nodes():
    lo0 = node.create_port(PortType.LOGICAL)
    node.set("lo0_id", lo0.id)

# assign port labels
for node in t_phy.nodes():
    physical_ports = filters.physical_ports(node)
    for index, port in enumerate(physical_ports):
        port.set("label", f"GigabitEthernet{index}/0")

t_ip = network_model.create_topology("ip")
t_ip.add_nodes_from(t_phy.nodes())
t_ip.add_links_from(t_phy.links())
grouped = group_by(t_ip.nodes(), "asn")
for asn, nodes in grouped.items():
    for index, node in enumerate(nodes):
        lo0 = node.loopback_zero()
        loopback_ip = f"1.16.{asn}.{index+1}"
        lo0.set("ip", loopback_ip)
        dRouter_id[node.label] = loopback_ip

    links = [l for l in t_ip.links()
             if l.n1.get("asn") == l.n2.get("asn") == asn]
    for index, link in enumerate(links):
        prefix = f"1.{asn}.{index}"
        network = prefix + ".0"
        # print(network)
        link.p1.set("ip", prefix + ".1")
        link.p1.set("network", network)
        link.p2.set("ip", prefix + ".2")
        link.p2.set("network", network)

# inter-as links
links = [l for l in t_ip.links()
         if l.n1.get("asn") != l.n2.get("asn")]
for index, link in enumerate(links):
    prefix = f"1.0.{index}"
    network = prefix + ".0"
    # print(network)
    link.p1.set("ip", prefix + ".1")
    link.p1.set("network", network)
    link.p2.set("ip", prefix + ".2")

t_ospf = network_model.create_topology("ospf")
t_ospf.add_nodes_from(routers)
ebgp_links = [l for l in t_phy.links()
              if l.n1.get("asn") == l.n2.get("asn")]
t_ospf.add_links_from(ebgp_links)

# t_ibgp = network_model.create_topology("ibgp")
# t_ibgp.add_nodes_from(routers)
# ibgp_pairs = [(n1, n2) for n1 in t_ibgp.nodes()
#               for n2 in t_ibgp.nodes()
#               if n1 != n2 and n1.get("asn") == n2.get("asn")]
# for n1, n2 in ibgp_pairs:
#     p1 = n1.loopback_zero()
#     p2 = n2.loopback_zero()
#     t_ibgp.create_link(p1, p2)

# t_ebgp = network_model.create_topology("ebgp")
# t_ebgp.add_nodes_from(routers)
# ebgp_links = [l for l in t_phy.links()
#               if l.n1.get("asn") != l.n2.get("asn")]
# t_ebgp.add_links_from(ebgp_links)

# analysis
import networkx as nx

graph = topology_to_nx_graph(t_phy)
path = nx.shortest_path(graph)
path = wrap_node_ids(t_phy, path)

p1 = t_phy.create_node_path(path)

# Compile device models
compiled = {}

for node in filters.routers(t_phy):
    data = {
        "hostname": node.label,
        "interfaces": [],
        "router_id": dRouter_id.get(node.label),
        "asn": node.get("asn")
    }
    for port in filters.physical_ports(node):
        ip_port = t_ip.get_port_by_id(port.id)
        data["interfaces"].append({
            "id": port.label,
            "ip": ip_port.get("ip")
        })

    # for asn, nodes in grouped.items():
    #     for index, node in enumerate(nodes):
    #         lo0 = node.loopback_zero()
    # lo0 = node.loopback_zero();
    # data["router_id"] = lo0.get("ip");


    ospf_node = t_ospf.get_node_by_id(node.id)
    ospf_enabled = ospf_node.degree() > 0
    data["ospf"] = {"networks": [],
                    "enabled":ospf_enabled}

    for port in filters.physical_ports(ospf_node):
        if not port.connected:
            continue
        ip_port = t_ip.get_port_by_id(port.id)
        network = ip_port.get("network")
        data["ospf"]["networks"].append(network)


    # ebgp_node = t_ebgp.get_node_by_id(node.id)
    # data["ebgp"] = {"neighbors": []}
    # for peer in ebgp_node.peer_nodes():
    #     ip_peer = t_ip.get_node_by_id(peer.id)
    #     peer_ip = ip_peer.loopback_zero().get("ip")
    #     data["ebgp"]["neighbors"].append({
    #         "ip": peer_ip,
    #         "asn": peer.get("asn")
    #     })

    # ibgp_node = t_ibgp.get_node_by_id(node.id)
    # bgp_enabled = ebgp_node.degree() > 0 or ibgp_node.degree() > 0
    # data["bgp_enabled"] = bgp_enabled
    # data["ibgp"] = {"neighbors": []}
    # for peer in ibgp_node.peer_nodes():
    #     ip_peer = t_ip.get_node_by_id(peer.id)
    #     peer_ip = ip_peer.loopback_zero().get("ip")
    #     data["ibgp"]["neighbors"].append({
    #         "ip": peer_ip,
    #         "asn": peer.get("asn")
    #     })

    compiled[node] = data

for node in filters.hosts(t_phy):
    data = {
        "hostname": node.label,
        "interfaces": []
    }

    for port in filters.physical_ports(node):
        ip_port = t_ip.get_port_by_id(port.id)
        data["interfaces"].append({
            "id": port.label,
            "ip": ip_port.get("ip")
        })

    compiled[node] = data

# and render using template
rtr_template_str = """
!
hostname {{ data.hostname }}
!
interface Loopback0
 ip address {{data.router_id}} 255.255.255.255
!
{% for interface in data.interfaces %}
!
interface {{interface.id}}
  ip address {{ interface.ip}} 255.255.255.0
  negotiation auto
!
{% endfor %}
{% if data.ospf.enabled %}
!
router ospf 1
  router-id {{data.router_id}}
{% for network in data.ospf.networks %}
  network {{network}} 0.255.255.255 area 1
{% endfor %}
!
{% endif %}
!
"""
# host_template_str = """
# ! host
# hostname {{ data.hostname }}
# {% for interface in data.interfaces %}
# {{interface.id}} {{ interface.ip}} up
# {% endfor %}
# """

templates = {
    DeviceType.ROUTER: Template(rtr_template_str, trim_blocks=True)
    # DeviceType.HOST: Template(host_template_str, trim_blocks=True)
}

for node, data in compiled.items():
    template = templates[node.type]
    rendered = template.render(data=data)
    # fp = open("/home/zzh/nsdi22/pybatfish/jupyter_notebooks/networks/ospf-test-49nodes/configs/"+ node.label + ".cfg", "w")
    # fp.write(rendered)
    # fp.close()
    print(rendered)

publish_model_to_webserver(network_model)
