#!/usr/bin/env python
#!-*- coding: utf-8 -*-
import pandas as pd
from collections import deque
import math

# 정보 불러오기
node = pd.read_csv('./data/node_data.csv', keep_default_na=False)
link = pd.read_csv('./data/A2_link.csv', keep_default_na=False)
tracking = pd.read_csv('./data/tracking.csv', keep_default_na=False)

# 노드 생성

class Node:
  def __init__(self, parent, position, name):
    self.parent = parent # 이전 노드 ID
    self.position = position # 좌표
    self.name = name # 현재 노드 ID

    self.g = 0 # g = 현재 노드에서 출발 지점까지의 총 cost
    self.h = 0 # h = 휴리스틱, 현재 노드에서 목적지까지의 추정 거리
    self.f = 0 # f = 출발 지점에서 목적지까지의 총 cost 합

  def __eq__(self, other):
    return self.position == other.position

# 노드, 링크 정보 담기

def make_info():
  '''
  # 4M 간격 Data Practice
  # 노드 ID순 'X좌표', 'Y좌표' 
  for i in range(0,len(node), 2):
    node_dict[node['ID'][i]] = [node['xcoord'][i], node['ycoord'][i]]

  # 링크 ID순 '시작노드', '끝노드', '비용', '자신의 링크 ID'
  for i in range(0, len(link), 2):
    link_dict[link['ID'][i]] = [link['FromNodeID'][i], link['ToNodeID'][i], link['Length'][i], link['ID'][i]]

  # 노드가 속해있는 링크의 ID순 '링크의 시작노드', '링크의 끝노드', 'FromNodeID 부터의 거리', 'Node의 X좌표', 'Node의 Y좌표'
  for i in range(0, len(tracking), 2):
    if tracking['ID'][i] not in tracking_dict:
      tracking_dict[tracking['ID'][i]] = [[tracking['FromNodeID'][i], tracking['ToNodeID'][i], tracking['distance'][i], tracking['xcoord'][i], tracking['ycoord'][i]]]

    else: 
      tracking_dict[tracking['ID'][i]].append([tracking['FromNodeID'][i], tracking['ToNodeID'][i], tracking['distance'][i], tracking['xcoord'][i], tracking['ycoord'][i]])
 '''

  # 2M 간격 Data
  # 노드 ID순 'X좌표', 'Y좌표'
  for i in range(len(node)):
    node_dict[node['ID'][i]] = [node['xcoord'][i], node['ycoord'][i]]

  # 링크 ID순 '시작노드', '끝노드', '비용', '자신의 링크 ID'
  for i in range(len(link)):
    link_dict[link['ID'][i]] = [link['FromNodeID'][i], link['ToNodeID'][i], link['Length'][i], link['ID'][i]]

  # 노드가 속해있는 링크의 ID순 '링크의 시작노드', '링크의 끝노드', 'FromNodeID 부터의 거리', 'Node의 X좌표', 'Node의 Y좌표'
  for i in range(len(tracking)):
    if tracking['ID'][i] not in tracking_dict:
      tracking_dict[tracking['ID'][i]] = [[tracking['FromNodeID'][i], tracking['ToNodeID'][i], tracking['distance'][i], tracking['xcoord'][i], tracking['ycoord'][i]]]

    else:
      tracking_dict[tracking['ID'][i]].append([tracking['FromNodeID'][i], tracking['ToNodeID'][i], tracking['distance'][i], tracking['xcoord'][i], tracking['ycoord'][i]])

# Diagonal Distance

def heuristic(node, goal, D=1, D2=2 ** 0.5):
    dx = abs(node.position[0] - goal.position[0])
    dy = abs(node.position[1] - goal.position[1])

    return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)

# A* algorithm

def A_star(startNode, endNode):
    # startNode와 endNode 초기화
    startNode = Node(None, startNode[:2], startNode[2])
    endNode = Node(None, endNode[:2], endNode[2])

    # openList, closedList 초기화
    closedList = []
    openList = [startNode] # 시작노드 추가

    # endNode를 찾을 때 까지 반복
    while openList:

      # 현재 위치하는 노드 지정
      currentNode = openList[0]
      currentIdx = 0

      # 이미 같은 노드가 openList안에 있고, openList안에 있는 node의 f값이 더 크면
      # currentNode를 openList안에 있는 node로 교체
      # why? 최단 f를 찾는 것이기 때문에, 더 작은 f값을 closedList에 넣는다.
      for idx, node in enumerate(openList):
          if node.f < currentNode.f: # 이미 방문하여 openList안에 있는 node < 현재 위치하고 있는 node
              currentNode = node
              currentIdx = idx

      # 유효한 데이터 임으로 openList에서 제거하고 closedList에 추가
      openList.pop(currentIdx)
      closedList.append(currentNode)

      # 현재 노드가 목적지면 current.position에 추가하고
      # current 부모로 이동
      if currentNode.name == endNode.name:
        path = []
        current = currentNode

        while current is not None:
          path.append(current.position)

          for i in reversed(range(len(tracking))):
            if tracking['ToNodeID'][i] == current.name and tracking['FromNodeID'][i] == current.parent.name:
              if tracking['distance'][i] == 0:
                continue

              path.append((tracking['xcoord'][i], tracking['ycoord'][i], tracking['distance'][i]))

          if current.parent.name == startNode.name:
            break

          current = current.parent

        path.append(startNode.position)
        return path[::-1]

      tmp = deque() # 중복을 방지하기 위해 set()함수 사용
      arr = []

      for f in link_dict.values():
        if f[0] == currentNode.name: # 유효 링크들 추가 링크의 ID들을 tmp에 담는다.
          tmp.appendleft(f[3])

      # 현재 link의 Length 정보들 배열에 추가
      for current_link in tmp: # 링크에 대한 정보
        arr.append(float(link_dict[current_link][2]))

      children = [] # 위치를 담을 배열
      for newPosition in tmp: # link_dict[newPosition][4] = 자신 링크의 ID
        if link_dict[newPosition][1] in node_dict:
          nodePosition = (node_dict[link_dict[newPosition][1]][0], node_dict[link_dict[newPosition][1]][1]) # 링크의 ToNodeID의 x, y 좌표
          new_node = Node(currentNode, nodePosition, link_dict[newPosition][1]) # 이전 노드는 부모 노드, 새로운 노드의 위치, 링크의 ToNodeID
          children.append(new_node)

        else:
          continue

      for child, link_len in zip(children, arr):
        if child in closedList:
          continue

        # f,g,h 값 갱신
        child.g = currentNode.g + link_len
        child.h = ((child.position[0] - endNode.position[0]) ** 2) + ((child.position[1] - endNode.position[1]) ** 2)
        child.f = child.g + child.h

        if len([openNode for openNode in openList if child == openNode and child.g > openNode.g]) > 0:
          continue

        openList.append(child)

if __name__ == '__main__':
  node_dict = {}
  link_dict = {}
  tracking_dict = {}
  make_info()

  # node 와 link의 ID는 다르다
  # 출발점은 출발하고 싶은 Node의 link 'ID'에 해당하는 FromNode를 시작노드로 설정한다.
  # node_dict 에 설정한 FromNodeID를 담아서 X좌표, Y좌표를 설정한다.

  # 도착점은 도착하고 싶은 Node의 link 'ID'에 해당하는 ToNodeID를 시작노드로 설정한다.
  # A_star(node_dict[link_dict['ID'][0]][0], node_dict[link_dict['ID'][0]][1])

  # 시작 노드의 X좌표, Y좌표, 링크 ID

  '''
  K_city_waypoint = deque([])

  for i in range(node_num): # node_num is variable
    start, end = input().split()

    start_node = [node_dict[link_dict[start][0]][0], node_dict[link_dict[start][0]][1], link_dict[start][0]]
    end_node = [node_dict[link_dict[end][1]][0], node_dict[link_dict[end][1]][1], link_dict[end][1]]

    K_city_waypoint.append([start_node, end_node])
  '''


  #About K-City
  K_city_waypoint = deque([
                           [(node_dict[link_dict['A219BS010380'][0]][0], node_dict[link_dict['A219BS010380'][0]][1], link_dict['A219BS010380'][0]),
                            (node_dict[link_dict['A219BS010394'][1]][0], node_dict[link_dict['A219BS010394'][1]][1], link_dict['A219BS010394'][1])],
                           [(node_dict[link_dict['A219BS010659'][0]][0], node_dict[link_dict['A219BS010659'][0]][1], link_dict['A219BS010659'][0]),
                            (node_dict[link_dict['A219BS010387'][1]][0], node_dict[link_dict['A219BS010387'][1]][1], link_dict['A219BS010387'][1])],
                           [(node_dict[link_dict['A219BS010643'][0]][0], node_dict[link_dict['A219BS010643'][0]][1], link_dict['A219BS010643'][0]),
                            (node_dict[link_dict['A2_remark115'][1]][0], node_dict[link_dict['A2_remark115'][1]][1], link_dict['A2_remark115'][1])],
                           [(node_dict[link_dict['A2_remark113'][0]][0], node_dict[link_dict['A2_remark113'][0]][1], link_dict['A2_remark113'][0]),
                            (node_dict[link_dict['A2_remark090'][1]][0], node_dict[link_dict['A2_remark090'][1]][1], link_dict['A2_remark090'][1])],
                           [(node_dict[link_dict['A219BS010084'][0]][0], node_dict[link_dict['A219BS010084'][0]][1], link_dict['A219BS010084'][0]),
                            (node_dict[link_dict['A219BS010377'][1]][0], node_dict[link_dict['A219BS010377'][1]][1], link_dict['A219BS010377'][1])]
                          ])
  '''
  K_city_waypoint = deque([
                           [(node_dict[link_dict['a'][0]][0], node_dict[link_dict['a'][0]][1], link_dict['a'][0]),
                            (node_dict[link_dict['i'][1]][0], node_dict[link_dict['i'][1]][1], link_dict['i'][1])],
                           [(node_dict[link_dict['j'][0]][0], node_dict[link_dict['j'][0]][1], link_dict['j'][0]),
                            (node_dict[link_dict['f'][1]][0], node_dict[link_dict['f'][1]][1], link_dict['f'][1])],
                           [(node_dict[link_dict['g'][0]][0], node_dict[link_dict['g'][0]][1], link_dict['g'][0]),
                            (node_dict[link_dict['a'][1]][0], node_dict[link_dict['a'][1]][1], link_dict['a'][1])],
                          ])
 '''
  coordinate_CSV = []
  for i in range(len(K_city_waypoint)):
    start, end = K_city_waypoint.popleft()
    path = A_star(start, end)

    # 좌표들 CSV 파일 형태로 만들어서 저장하기
    # X좌표, Y좌표 추가
    for i in path:
      coordinate_CSV.append([i[0], i[1]])

    # A* algorithm 실행해서 나온 좌표들 CSV 파일화
  coordinate_CSV = pd.DataFrame(coordinate_CSV, columns=["X","Y"])
  print(coordinate_CSV)
  coordinate_CSV.to_csv("/home/car/test_ERP42/src/path_planning/scripts/path.csv")
