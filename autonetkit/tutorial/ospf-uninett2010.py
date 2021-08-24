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

for num in range(0, 74):
    r[num] = t_phy.create_node(DeviceType.ROUTER, "r" + str(num))
# r5 = t_phy.create_node(DeviceType.ROUTER, "r5")
# h1 = t_phy.create_node(DeviceType.HOST, "h1")
# h2 = t_phy.create_node(DeviceType.HOST, "h2")
dRouter_id = {}

# r0.set("x", 0)
# r0.set("y", 0)
# r0.set("asn", 1)

properties = {}

for num in range(0, 74):
    properties.setdefault("r"+ str(num), (10,20,100));
    # if num<=255:
    #     properties.setdefault("r"+ str(num), (10,20,100));
    # if 255<num<510:
    #     properties.setdefault("r" + str(num), (15, 25, 150));
    # else:
    #     properties.setdefault("r" + str(num), (20, 25, 200));

for node_id, (x, y, asn) in properties.items():
    node = find_node_by_label(t_phy, node_id)
    node.set("x", x)
    node.set("y", y)
    node.set("asn", asn)


# or in a loop

# pairs = [(r[0], r[64]), (r[0], r[1]), (r[0], r[9]), (r[0], r[8]), (r[0], r[63]), (r[1], r[65]),(r[2], r[55]), (r[2], r[87]),
#          (r[3], r[88]), (r[3], r[4]), (r[3], r[45]), (r[3], r[86]), (r[3], r[47]), (r[4], r[86]),
#          (r[4], r[5]), (r[4], r[6]), (r[4], r[7]), (r[5], r[46]), (r[6], r[80]), (r[6], r[81]),
#          (r[6], r[62]), (r[7], r[8]), (r[8], r[86]), (r[8], r[63]), (r[9], r[63]), (r[10], r[16]),
#          (r[10], r[104]), (r[10], r[11]), (r[10], r[13]), (r[11], r[16]), (r[11], r[104]), (r[11], r[13]),
#          (r[12], r[19]), (r[12], r[13]), (r[14], r[17]), (r[14], r[84]), (r[15], r[97]), (r[15], r[47]),
#          (r[16], r[104]), (r[17], r[43]), (r[18], r[19]), (r[18], r[35]), (r[19], r[104]), (r[19], r[36]), (r[19], r[37]),
#          (r[20], r[21]), (r[20], r[22]), (r[20], r[23]), (r[21], r[26]), (r[22], r[83]),
#          (r[23], r[105]), (r[24], r[25]), (r[24], r[27]), (r[24], r[95]), (r[25], r[89]), (r[25], r[26]),
#          (r[25], r[28]), (r[25], r[92]), (r[27], r[83]), (r[28], r[29]), (r[29], r[92]), (r[30], r[103]), (r[30], r[36]),
#          (r[30], r[77]), (r[30], r[31]), (r[31], r[112]), (r[31], r[36]), (r[31], r[37]), (r[32], r[33]), (r[32], r[34]),
#          (r[33], r[38]), (r[34], r[35]), (r[36], r[37]), (r[38], r[39]), (r[40], r[41]), (r[40], r[49]), (r[41], r[42]),
#          (r[42], r[43]), (r[43], r[45]), (r[44], r[47]), (r[46], r[47]), (r[47], r[73]), (r[47], r[51]), (r[47], r[84]),
#          (r[47], r[111]), (r[47], r[60]), (r[48], r[57]), (r[48], r[49]), (r[49], r[84]), (r[50], r[57]), (r[50], r[91]),
#          (r[50], r[52]), (r[50], r[51]), (r[52], r[53]), (r[53], r[59]), (r[53], r[60]), (r[54], r[56]), (r[54], r[90]),
#          (r[54], r[111]), (r[54], r[55]), (r[54], r[94]), (r[55], r[111]), (r[56], r[57]), (r[58], r[105]), (r[58], r[61]),
#          (r[58], r[110]), (r[59], r[60]), (r[60], r[104]), (r[60], r[97]), (r[61], r[82]), (r[62], r[80]), (r[62], r[70]),
#          (r[62], r[85]), (r[63], r[85]), (r[63], r[70]), (r[64], r[65]), (r[64], r[66]), (r[64], r[67]), (r[66], r[67]),
#          (r[66], r[69]), (r[66], r[85]), (r[68], r[69]), (r[68], r[71]), (r[70], r[71]), (r[72], r[73]),
#          (r[72], r[75]), (r[72], r[81]), (r[73], r[98]), (r[74], r[112]), (r[74], r[75]), (r[74], r[103]),
#          (r[75], r[104]), (r[75], r[76]), (r[76], r[100]), (r[77], r[81]), (r[77], r[100]), (r[77], r[103]),
#          (r[78], r[101]), (r[79], r[102]), (r[81], r[101]), (r[82], r[83]), (r[87], r[88]), (r[88], r[93]),
#          (r[89], r[96]), (r[89], r[90]), (r[91], r[92]), (r[93], r[94]), (r[94], r[95]), (r[95], r[96]),
#          (r[97], r[98]), (r[98], r[99]), (r[99], r[104]), (r[99], r[112]), (r[100], r[102]), (r[101], r[102]),
#          (r[105], r[106]), (r[106], r[107]), (r[107], r[108]), (r[108], r[109]), (r[109], r[110]),
#          ]
edge = "edge"
source = "source"
target = "target"
fp = open("/home/zzh/Documents/networkTopology/Uninett2010.gml", "r")
text_line = fp.readlines();

pairs = []

for num in range(0,len(text_line)):
    currentLine = text_line[num];
    if(edge in currentLine):
        if(source in text_line[num+1]):
            if(target in text_line[num+2]):
                # print(text_line[num+1][11:((len(text_line[num+1])-1))])
                # print(text_line[num+2][11:((len(text_line[num+2])-1))])
                pairs.append((r[int(text_line[num+1][11:((len(text_line[num+1])-1))])],
                              r[int(text_line[num+2][11:((len(text_line[num+2])-1))])]))
fp.close()


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
        print(network)
        link.p1.set("ip", prefix + ".1")
        link.p1.set("network", network)
        link.p2.set("ip", prefix + ".2")
        link.p2.set("network", network)

# inter-as links
links = [l for l in t_ip.links()
         if l.n1.get("asn") != l.n2.get("asn")]
for index, link in enumerate(links):
    if(index<=255):
        prefix = f"1.0.{index}"
    elif(index<=510):
        prefix = f"1.1.{index-255}"
    else:
        prefix = f"1.2.{index-510}"
    network = prefix + ".0"
    print(network)
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
    fp = open("/home/zzh/nsdi22/pybatfish/jupyter_notebooks/networks/ospf-test-uninett2010/configs/"+ node.label + ".cfg", "w")
    fp.write(rendered)
    fp.close()
    # print(rendered)

publish_model_to_webserver(network_model)
