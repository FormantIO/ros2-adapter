from agent_mock import AgentMockServicer, serve

servicer = AgentMockServicer()
server = serve(servicer)
server.wait_for_termination()
