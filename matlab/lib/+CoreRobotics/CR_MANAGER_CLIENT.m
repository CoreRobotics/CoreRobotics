function v = CR_MANAGER_CLIENT()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 37);
  end
  v = vInitialized;
end
