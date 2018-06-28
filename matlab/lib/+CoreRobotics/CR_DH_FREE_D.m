function v = CR_DH_FREE_D()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 30);
  end
  v = vInitialized;
end
