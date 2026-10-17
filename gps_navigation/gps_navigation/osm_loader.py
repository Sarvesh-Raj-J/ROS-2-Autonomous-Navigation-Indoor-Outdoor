#!/usr/bin/env python3
"""
Shared OSM extract loader for XML `.osm` and binary `.pbf` files.

This loader is intentionally minimal: it only parses the parts of OSM PBF we
need for routing and static obstacle generation (nodes + ways + tags).
"""

import struct
import xml.etree.ElementTree as ET
import zlib


def zigzag_decode(value: int) -> int:
    return (value >> 1) ^ (-(value & 1))


def read_varint(data: bytes, offset: int):
    shift = 0
    value = 0
    while True:
        if offset >= len(data):
            raise RuntimeError('Unexpected end of data while reading varint')
        byte = data[offset]
        offset += 1
        value |= (byte & 0x7F) << shift
        if not (byte & 0x80):
            return value, offset
        shift += 7
        if shift > 70:
            raise RuntimeError('Varint is too large')


def skip_proto_field(data: bytes, offset: int, wire_type: int):
    if wire_type == 0:
        _, offset = read_varint(data, offset)
        return offset
    if wire_type == 1:
        return offset + 8
    if wire_type == 2:
        length, offset = read_varint(data, offset)
        return offset + length
    if wire_type == 5:
        return offset + 4
    raise RuntimeError(f'Unsupported protobuf wire type: {wire_type}')


def read_length_delimited(data: bytes, offset: int):
    length, offset = read_varint(data, offset)
    end = offset + length
    if end > len(data):
        raise RuntimeError('Length-delimited field exceeds buffer size')
    return data[offset:end], end


def read_packed_varints(data: bytes):
    values = []
    offset = 0
    while offset < len(data):
        value, offset = read_varint(data, offset)
        values.append(value)
    return values


def parse_blob_header(data: bytes):
    blob_type = None
    data_size = None
    offset = 0
    while offset < len(data):
        key, offset = read_varint(data, offset)
        field_number = key >> 3
        wire_type = key & 0x07
        if field_number == 1 and wire_type == 2:
            raw, offset = read_length_delimited(data, offset)
            blob_type = raw.decode('utf-8')
        elif field_number == 3 and wire_type == 0:
            data_size, offset = read_varint(data, offset)
        else:
            offset = skip_proto_field(data, offset, wire_type)
    if blob_type is None or data_size is None:
        raise RuntimeError('Incomplete OSM PBF blob header')
    return blob_type, data_size


def parse_blob(data: bytes):
    raw = None
    zlib_data = None
    offset = 0
    while offset < len(data):
        key, offset = read_varint(data, offset)
        field_number = key >> 3
        wire_type = key & 0x07
        if wire_type != 2:
            offset = skip_proto_field(data, offset, wire_type)
            continue
        payload, offset = read_length_delimited(data, offset)
        if field_number == 1:
            raw = payload
        elif field_number == 3:
            zlib_data = payload
    if raw is not None:
        return raw
    if zlib_data is not None:
        return zlib.decompress(zlib_data)
    raise RuntimeError('Unsupported OSM PBF blob compression')


def parse_string_table(data: bytes):
    strings = []
    offset = 0
    while offset < len(data):
        key, offset = read_varint(data, offset)
        field_number = key >> 3
        wire_type = key & 0x07
        if field_number == 1 and wire_type == 2:
            payload, offset = read_length_delimited(data, offset)
            strings.append(payload.decode('utf-8'))
        else:
            offset = skip_proto_field(data, offset, wire_type)
    return strings


def parse_dense_nodes(data: bytes, granularity: int, lat_offset: int, lon_offset: int):
    id_deltas = []
    lat_deltas = []
    lon_deltas = []
    offset = 0
    while offset < len(data):
        key, offset = read_varint(data, offset)
        field_number = key >> 3
        wire_type = key & 0x07
        if wire_type != 2:
            offset = skip_proto_field(data, offset, wire_type)
            continue
        payload, offset = read_length_delimited(data, offset)
        if field_number == 1:
            id_deltas = [zigzag_decode(value) for value in read_packed_varints(payload)]
        elif field_number == 8:
            lat_deltas = [zigzag_decode(value) for value in read_packed_varints(payload)]
        elif field_number == 9:
            lon_deltas = [zigzag_decode(value) for value in read_packed_varints(payload)]

    nodes = {}
    current_id = 0
    current_lat = 0
    current_lon = 0
    count = min(len(id_deltas), len(lat_deltas), len(lon_deltas))
    for idx in range(count):
        current_id += id_deltas[idx]
        current_lat += lat_deltas[idx]
        current_lon += lon_deltas[idx]
        lat = 1e-9 * (lat_offset + granularity * current_lat)
        lon = 1e-9 * (lon_offset + granularity * current_lon)
        nodes[current_id] = (lat, lon)
    return nodes


def parse_node(data: bytes, granularity: int, lat_offset: int, lon_offset: int):
    node_id = None
    lat_raw = None
    lon_raw = None
    offset = 0
    while offset < len(data):
        key, offset = read_varint(data, offset)
        field_number = key >> 3
        wire_type = key & 0x07
        if field_number == 1 and wire_type == 0:
            node_id, offset = read_varint(data, offset)
        elif field_number == 8 and wire_type == 0:
            value, offset = read_varint(data, offset)
            lat_raw = zigzag_decode(value)
        elif field_number == 9 and wire_type == 0:
            value, offset = read_varint(data, offset)
            lon_raw = zigzag_decode(value)
        else:
            offset = skip_proto_field(data, offset, wire_type)
    if node_id is None or lat_raw is None or lon_raw is None:
        return None
    lat = 1e-9 * (lat_offset + granularity * lat_raw)
    lon = 1e-9 * (lon_offset + granularity * lon_raw)
    return node_id, lat, lon


def parse_way(data: bytes, strings):
    way_id = None
    keys = []
    vals = []
    refs = []
    offset = 0
    while offset < len(data):
        key, offset = read_varint(data, offset)
        field_number = key >> 3
        wire_type = key & 0x07
        if field_number == 1 and wire_type == 0:
            way_id, offset = read_varint(data, offset)
        elif field_number in (2, 3) and wire_type == 2:
            payload, offset = read_length_delimited(data, offset)
            values = read_packed_varints(payload)
            if field_number == 2:
                keys = values
            else:
                vals = values
        elif field_number == 8 and wire_type == 2:
            payload, offset = read_length_delimited(data, offset)
            current_ref = 0
            for value in read_packed_varints(payload):
                current_ref += zigzag_decode(value)
                refs.append(current_ref)
        else:
            offset = skip_proto_field(data, offset, wire_type)

    tags = {}
    for idx in range(min(len(keys), len(vals))):
        key_idx = keys[idx]
        val_idx = vals[idx]
        if key_idx < len(strings) and val_idx < len(strings):
            tags[strings[key_idx]] = strings[val_idx]
    return {
        'id': way_id,
        'refs': refs,
        'tags': tags,
    }


def parse_primitive_group(
    data: bytes, strings, granularity: int, lat_offset: int, lon_offset: int
):
    nodes = {}
    ways = []
    offset = 0
    while offset < len(data):
        key, offset = read_varint(data, offset)
        field_number = key >> 3
        wire_type = key & 0x07
        if wire_type != 2:
            offset = skip_proto_field(data, offset, wire_type)
            continue
        payload, offset = read_length_delimited(data, offset)
        if field_number == 1:
            node = parse_node(payload, granularity, lat_offset, lon_offset)
            if node is not None:
                node_id, lat, lon = node
                nodes[node_id] = (lat, lon)
        elif field_number == 2:
            nodes.update(parse_dense_nodes(payload, granularity, lat_offset, lon_offset))
        elif field_number == 3:
            ways.append(parse_way(payload, strings))
    return nodes, ways


def parse_primitive_block(data: bytes):
    strings = []
    granularity = 100
    lat_offset = 0
    lon_offset = 0
    primitive_groups = []
    offset = 0

    while offset < len(data):
        key, offset = read_varint(data, offset)
        field_number = key >> 3
        wire_type = key & 0x07
        if field_number == 1 and wire_type == 2:
            payload, offset = read_length_delimited(data, offset)
            strings = parse_string_table(payload)
        elif field_number == 2 and wire_type == 2:
            payload, offset = read_length_delimited(data, offset)
            primitive_groups.append(payload)
        elif field_number == 17 and wire_type == 0:
            granularity, offset = read_varint(data, offset)
        elif field_number == 19 and wire_type == 0:
            lat_offset, offset = read_varint(data, offset)
        elif field_number == 20 and wire_type == 0:
            lon_offset, offset = read_varint(data, offset)
        else:
            offset = skip_proto_field(data, offset, wire_type)

    nodes = {}
    ways = []
    for group in primitive_groups:
        group_nodes, group_ways = parse_primitive_group(
            group,
            strings,
            granularity,
            lat_offset,
            lon_offset,
        )
        nodes.update(group_nodes)
        ways.extend(group_ways)
    return nodes, ways


def load_osm_pbf(path: str):
    nodes = {}
    ways = []
    with open(path, 'rb') as pbf_file:
        while True:
            header_len_raw = pbf_file.read(4)
            if not header_len_raw:
                break
            if len(header_len_raw) != 4:
                raise RuntimeError('Incomplete OSM PBF blob header length')
            header_len = struct.unpack('>I', header_len_raw)[0]
            header_data = pbf_file.read(header_len)
            if len(header_data) != header_len:
                raise RuntimeError('Incomplete OSM PBF blob header')
            blob_type, blob_size = parse_blob_header(header_data)
            blob_data = pbf_file.read(blob_size)
            if len(blob_data) != blob_size:
                raise RuntimeError('Incomplete OSM PBF blob')
            blob_payload = parse_blob(blob_data)
            if blob_type == 'OSMData':
                block_nodes, block_ways = parse_primitive_block(blob_payload)
                nodes.update(block_nodes)
                ways.extend(block_ways)
    return nodes, ways


def load_osm_xml(path: str):
    tree = ET.parse(path)
    root = tree.getroot()

    nodes = {}
    for node_elem in root.findall('node'):
        nodes[int(node_elem.attrib['id'])] = (
            float(node_elem.attrib['lat']),
            float(node_elem.attrib['lon']),
        )

    ways = []
    for way_elem in root.findall('way'):
        tags = {tag.attrib['k']: tag.attrib['v'] for tag in way_elem.findall('tag')}
        refs = [
            int(nd.attrib['ref'])
            for nd in way_elem.findall('nd')
            if int(nd.attrib['ref']) in nodes
        ]
        ways.append({
            'id': int(way_elem.attrib.get('id', '0')),
            'refs': refs,
            'tags': tags,
        })
    return nodes, ways


def load_osm_extract(path: str):
    lower_path = path.lower()
    if lower_path.endswith('.pbf'):
        return load_osm_pbf(path)
    return load_osm_xml(path)
