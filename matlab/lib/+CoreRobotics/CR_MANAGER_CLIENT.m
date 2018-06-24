function v = CR_MANAGER_CLIENT()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 35);
  end
  v = vInitialized;
end
